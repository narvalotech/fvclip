#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/pwm.h>

#include "serial.h"
#include "eeprom.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, 4);

#define FV1_PGM_SIZE 512
#define EEPROM_SIZE (FV1_PGM_SIZE * 2)
static uint8_t rom_data[EEPROM_SIZE];
static uint8_t active_id;

enum {
	PGM_BUILTIN = 0,
	PGM_EXTERNAL,
};

#define GPIO_S0_PIN DT_PHA(DT_PATH(outputs, gpio_s0), gpios, pin)
#define GPIO_S1_PIN DT_PHA(DT_PATH(outputs, gpio_s1), gpios, pin)
#define GPIO_S2_PIN DT_PHA(DT_PATH(outputs, gpio_s2), gpios, pin)
#define GPIO_EXT_PIN DT_PHA(DT_PATH(outputs, gpio_ext), gpios, pin)
#define GPIO_CLIP_PIN DT_PHA(DT_PATH(inputs, gpio_clip), gpios, pin)

void init_gpios(void)
{
	const struct device *port =
		DEVICE_DT_GET(DT_PHANDLE(DT_PATH(outputs, gpio_s0), gpios));

	uint32_t out_flags = GPIO_OUTPUT;
	uint32_t in_flags = GPIO_INPUT;

	gpio_pin_configure(port, GPIO_S0_PIN, out_flags);
	gpio_pin_configure(port, GPIO_S1_PIN, out_flags);
	gpio_pin_configure(port, GPIO_S2_PIN, out_flags);
	gpio_pin_configure(port, GPIO_EXT_PIN, out_flags);

	gpio_pin_configure(port, GPIO_CLIP_PIN, in_flags);
}

void select_program(uint8_t id)
{
	const struct device *port =
		DEVICE_DT_GET(DT_PHANDLE(DT_PATH(outputs, gpio_s0), gpios));

	if (id > 7) {
		LOG_ERR("Program ID out of bounds (> 7)");
		return;
	}

	LOG_INF("select program %u", id);
	gpio_pin_set(port, GPIO_S0_PIN, id & 1);
	gpio_pin_set(port, GPIO_S1_PIN, id & 2);
	gpio_pin_set(port, GPIO_S2_PIN, id & 4);
}

void select_program_source(bool ext)
{
	const struct device *port =
		DEVICE_DT_GET(DT_PHANDLE(DT_PATH(outputs, gpio_s0), gpios));

	gpio_pin_set(port, GPIO_EXT_PIN, ext);
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
	select_program_source(PGM_EXTERNAL);
	select_program(id);
	LOG_INF("activated new program");
}

RING_BUF_DECLARE(uart_ringbuf, 1024);
K_SEM_DEFINE(serial_data, 0, 1);
static uint8_t serial_packet[FV1_PGM_SIZE];

void set_pwm_channel(uint8_t ch, uint16_t val)
{
	const struct device *pwm_dev = DEVICE_DT_GET(DT_ALIAS(pwm_0));
	uint32_t period = 1024;	/* 10-bit DAC */

	val = MIN(val, period);
	ch = MIN(ch, 2);

	pwm_set(pwm_dev, ch, PWM_NSEC(period * 10), PWM_NSEC(val * 10), 0);
	LOG_DBG("set pwm[%d]: %d", ch, val);
}

void init_pwm(void)
{
	set_pwm_channel(0, 0);
	set_pwm_channel(1, 0);
	set_pwm_channel(2, 0);
}

void load_from_serial(struct ring_buf *ringbuf, uint16_t len)
{
	/* Saturate length to dest buffer */
	len = MIN(len, sizeof(serial_packet));

	LOG_DBG("store ringbuf len %u splen %u", len, sizeof(serial_packet));
	ring_buf_get(ringbuf, serial_packet, len);
	pad_program(&serial_packet[len], FV1_PGM_SIZE - len);

	LOG_HEXDUMP_DBG(serial_packet, sizeof(serial_packet), "buffer");
	k_sem_give(&serial_data);
}

void set_pot_values(struct ring_buf *ringbuf, uint16_t len)
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

void print_bad_data(struct ring_buf *ringbuf, uint16_t len)
{
	static char tmp[1000] = {0};

	len = MIN(len, sizeof(tmp));
	ring_buf_get(ringbuf, tmp, len);
	LOG_HEXDUMP_WRN(tmp, sizeof(tmp), "unexpected opcode");
}

static void process_serial(struct rx_uart *config)
{
	switch (config->header->opcode) {
		case OP_LOAD:
			load_from_serial(config->ringbuf, config->header->len);
			break;
		case OP_POT:
			set_pot_values(config->ringbuf, config->header->len);
			break;
		default:
			LOG_ERR("unexpected opcode 0x%x", config->header->opcode);
			print_bad_data(config->ringbuf, config->header->len);
			break;
	}
}

static struct rx_uart_header uart_header;
static struct rx_uart config = {
	.uart = DEVICE_DT_GET(DT_NODELABEL(uart0)),
	.header = &uart_header,
	.ringbuf = &uart_ringbuf,
	.cb = process_serial,
};

void main(void)
{
	LOG_ERR("Bootup");

	init_pwm();
	serial_init(&config);

	init_i2c();
	init_eeprom(rom_data, FV1_PGM_SIZE);
	init_gpios();

	while (1) {
		/* Wait until we have received fresh data over serial */
		k_sem_take(&serial_data, K_FOREVER);
		load_program(serial_packet, sizeof(serial_packet));
	}
}
