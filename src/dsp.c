#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include <nrfx_ppi.h>
#include <nrfx_gpiote.h>

#include "dsp.h"
#include "utils.h"
#include "eeprom.h"
#include "rom.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dsp, 4);

#define FV1_PGM_SIZE 512
#define EEPROM_SIZE (FV1_PGM_SIZE * 2)
static uint8_t rom_data[EEPROM_SIZE];
static uint8_t active_id;

enum {
	PGM_BUILTIN = 0,
	PGM_EXTERNAL,
};

static void setup_dsp_clock(void)
{
	#define FVCLKPIN 22	/* P0.22 -> DSP GP5 -> FV1 X1 */
	#define MYTIMER NRF_TIMER3 /* Hopefully free? */
	#define CLK_HP_US 26   /* ~19kHz TODO: increase */
	#define GPIOTE_CH_INVALID 0xFF
	#define PPI_CH_INVALID 0xFF

	/* Use TIMER + PPI + GPIOTE to synthesize a 48kHz clock on DSP GP5. We
	 * have to play ball and use the HAL allocation fns for PPI / GPIOTE
	 * since we're running in an RTOS.
	 */

	/* step 1: configure TIMER */
	MYTIMER->BITMODE = 0;           /* 16-bit width */
	MYTIMER->PRESCALER = 4;         /* 1 MHz - 1 us */
	MYTIMER->MODE = 0;              /* Timer mode */
	MYTIMER->SHORTS = 1 << 0; /* CC[0] resets the timer value */
	MYTIMER->CC[0] = CLK_HP_US; /* one tick = half clock cycle */

	/* step 2: configure GPIOTE */
	uint8_t gch = GPIOTE_CH_INVALID;
	uint32_t err = nrfx_gpiote_channel_alloc(&gch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
	__ASSERT_NO_MSG(gch != GPIOTE_CH_INVALID);
	NRF_GPIOTE->CONFIG[gch] = 3; /* Task mode */
	NRF_GPIOTE->CONFIG[gch] |= FVCLKPIN << 8;
	NRF_GPIOTE->CONFIG[gch] |= 0 << 13; /* Port 0 (line not necessary for P0) */
	NRF_GPIOTE->CONFIG[gch] |= 3 << 16; /* Toggle pin on TASKS_OUT */
	NRF_GPIOTE->CONFIG[gch] |= 0 << 20; /* Initial pin value is LOW */

	/* step 3: configure PPI link */
	uint8_t pch = PPI_CH_INVALID;
	err = nrfx_ppi_channel_alloc(&pch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
	__ASSERT_NO_MSG(pch != PPI_CH_INVALID);

	NRF_PPI->CH[pch].EEP = (uint32_t)&MYTIMER->EVENTS_COMPARE[0];
	NRF_PPI->CH[pch].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[gch];
	NRF_PPI->CHENSET = 1 << pch;

	/* step 4: start the timer and output clock on pin */
	MYTIMER->TASKS_CLEAR = 1;
	MYTIMER->TASKS_START = 1;
}

static void init_gpios(void)
{
	uint32_t out_flags = GPIO_OUTPUT;

	/* configure FV-1 program selector pins */
	gpio_pin_configure(PORT_PIN(outputs, gpio_s0), out_flags);
	gpio_pin_configure(PORT_PIN(outputs, gpio_s1), out_flags);
	gpio_pin_configure(PORT_PIN(outputs, gpio_s2), out_flags);
}

static void set_pwm_channel(uint8_t ch, uint16_t val)
{
	const struct device *pwm_dev = DEVICE_DT_GET(DT_ALIAS(pwm_0));
	uint32_t period = 1024;	/* 10-bit DAC */

	val = MIN(val, period);
	ch = MIN(ch, 2);

	pwm_set(pwm_dev, ch, PWM_NSEC(period * 10), PWM_NSEC(val * 10), 0);
	LOG_DBG("set pwm[%d]: %d", ch, val);
}

static void init_pwm(void)
{
	set_pwm_channel(0, 0);
	set_pwm_channel(1, 0);
	set_pwm_channel(2, 0);
}

void dsp_set_pot_values(struct ring_buf *ringbuf, uint16_t len)
{
	char tmp[6] = {0};

	len = MIN(len, sizeof(tmp));
	ring_buf_get(ringbuf, tmp, len);

	__ASSERT(len == 6, "wrong length");

	set_pwm_channel(0, (tmp[0] << 8) + tmp[1]);
	set_pwm_channel(1, (tmp[2] << 8) + tmp[3]);
	set_pwm_channel(2, (tmp[4] << 8) + tmp[5]);

	LOG_HEXDUMP_DBG(tmp, sizeof(tmp), "pot values");
}

static void select_program(uint8_t id)
{
	if (id > 7) {
		LOG_ERR("Program ID out of bounds (> 7)");
		return;
	}

	LOG_INF("select program %u", id);
	gpio_pin_set(PORT_PIN(outputs, gpio_s0), id & 1);
	gpio_pin_set(PORT_PIN(outputs, gpio_s1), id & 2);
	gpio_pin_set(PORT_PIN(outputs, gpio_s2), id & 4);
}

static inline void pad_program(uint8_t * rom_addr, uint16_t pad_bytes)
{
	if (pad_bytes == 0) {
		LOG_INF("skip padding");
		return;
	}

	uint8_t * end_addr = rom_addr + pad_bytes;
	uint8_t nop[4] = {0x0, 0x0, 0x0, 0x11};

	while (rom_addr < end_addr) {
		memcpy(rom_addr, nop, sizeof(nop));
		rom_addr += 4;
	}
	LOG_INF("padded %u bytes", pad_bytes);
}

static void load_program(uint8_t * data, size_t bytes)
{
	/* Write to the program bank not in use */
	uint8_t id = active_id ? 0 : 1;
	active_id = id;

	__ASSERT_NO_MSG(bytes <= FV1_PGM_SIZE);

	uint8_t *rom_addr = rom_data + (id * FV1_PGM_SIZE);
	memcpy(rom_addr, data, bytes);

	LOG_INF("loaded %p into ROM bank %u", (void*)data, id);

	/* Tell EEPROM emulator to use the new bank */
	init_eeprom(rom_addr, FV1_PGM_SIZE);

	/* Trigger a reload from the FV-1 */
	select_program(id);
	LOG_INF("activated new program");
}

static uint8_t program[FV1_PGM_SIZE];

void dsp_load_from_serial(struct ring_buf *ringbuf, uint16_t len)
{
	/* Saturate length to dest buffer */
	len = MIN(len, sizeof(program));

	LOG_DBG("store ringbuf len %u splen %u", len, sizeof(program));
	ring_buf_get(ringbuf, program, len);
	pad_program(&program[len], FV1_PGM_SIZE - len);
	load_program(program, sizeof(program));

	LOG_HEXDUMP_DBG(program, sizeof(program), "buffer");
}

void init_dsp(void)
{
	LOG_DBG("");

	init_pwm();
	init_gpios();

	init_i2c();

	setup_dsp_clock();
	k_msleep(50);

	load_program((uint8_t *)samples_00, sizeof(samples_00));

	LOG_INF("DSP init ok");
}
