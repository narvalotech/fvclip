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
static uint8_t programs[10][FV1_PGM_SIZE];
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

void dsp_set_pwm_channel(uint8_t ch, uint16_t val)
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
	dsp_set_pwm_channel(0, 0);
	dsp_set_pwm_channel(1, 0);
	dsp_set_pwm_channel(2, 0);
}

void dsp_set_pot_values(struct ring_buf *ringbuf, uint16_t len)
{
	char tmp[6] = {0};

	len = MIN(len, sizeof(tmp));
	ring_buf_get(ringbuf, tmp, len);

	__ASSERT(len == 6, "wrong length");

	dsp_set_pwm_channel(0, (tmp[0] << 8) + tmp[1]);
	dsp_set_pwm_channel(1, (tmp[2] << 8) + tmp[3]);
	dsp_set_pwm_channel(2, (tmp[4] << 8) + tmp[5]);

	LOG_HEXDUMP_DBG(tmp, sizeof(tmp), "pot values");
}

static void select_rom_slot(uint8_t id)
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

static void load_program(uint8_t * data)
{
	/* This ID shuffling is just make the FV-1 read the new ROM */
	uint8_t id = active_id ? 0 : 1;
	active_id = id;

	LOG_INF("loading %p", (void*)data);

	/* Tell EEPROM emulator to use the new bank */
	init_eeprom(data, FV1_PGM_SIZE);

	/* Trigger a reload from the FV-1 */
	select_rom_slot(active_id);
	LOG_INF("program downloaded");
	/* That's not really true, it's downloaded later over I2C */
}

void dsp_load_from_serial(struct ring_buf *ringbuf, uint16_t len)
{
	/* Saturate length to dest buffer */
	len = MIN(len, FV1_PGM_SIZE);

	/* first byte is destination slot */
	uint8_t index;
	ring_buf_get(ringbuf, &index, 1);
	__ASSERT_NO_MSG(index < 10);

	/* TODO: protect with sem/mutex */
	uint8_t *program = &programs[index][0];

	/* TODO: add two opcodes:
	 * - load-raw: load raw program data as current program
	 * - load: load prog, with metadata:
	 *   - parameter names
	 *   - parameter default values
	 *   - program name
	 *   - destination slot
	 */
	LOG_DBG("store ringbuf idx %u len %u splen %u", index, len, FV1_PGM_SIZE);
	ring_buf_get(ringbuf, program, len);
	pad_program(&program[len], FV1_PGM_SIZE - len);

	load_program(program);

	LOG_HEXDUMP_DBG(program, FV1_PGM_SIZE, "RAW ROM");
}

void dsp_select_program(uint8_t id)
{
	if (id > 9) return;

	load_program(&programs[id][0]);
}

static void load_default_programs(void)
{
	memcpy(&programs[0][0], samples_00, FV1_PGM_SIZE);
	memcpy(&programs[1][0], samples_01, FV1_PGM_SIZE);
	memcpy(&programs[2][0], samples_02, FV1_PGM_SIZE);
	memcpy(&programs[3][0], samples_03, FV1_PGM_SIZE);
	memcpy(&programs[4][0], samples_04, FV1_PGM_SIZE);
	memcpy(&programs[5][0], samples_05, FV1_PGM_SIZE);
	memcpy(&programs[6][0], samples_06, FV1_PGM_SIZE);
	memcpy(&programs[7][0], samples_07, FV1_PGM_SIZE);
}

void init_dsp(void)
{
	LOG_DBG("");

	load_default_programs();

	init_pwm();
	init_gpios();

	init_i2c();

	/* prepare the ROM buffer that the FV1 will read on boot */
	load_program((uint8_t *)samples_00);

	setup_dsp_clock();
	k_msleep(50);

	LOG_INF("DSP init ok");
}
