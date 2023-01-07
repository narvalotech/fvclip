#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

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

void data_received(struct ring_buf *ringbuf, uint16_t len)
{
	/* Saturate length to dest buffer */
	len = MIN(len, sizeof(serial_packet));

	LOG_DBG("store ringbuf len %u splen %u", len, sizeof(serial_packet));
	ring_buf_get(ringbuf, serial_packet, len);
	pad_program(&serial_packet[len], FV1_PGM_SIZE - len);

	LOG_HEXDUMP_DBG(serial_packet, sizeof(serial_packet), "buffer");
	k_sem_give(&serial_data);
}

static struct rx_uart_header uart_header;
static struct rx_uart config = {
	.uart = DEVICE_DT_GET(DT_NODELABEL(uart0)),
	.header = &uart_header,
	.ringbuf = &uart_ringbuf,
	.cb = data_received,
};

void main(void)
{
	LOG_ERR("Bootup");

	serial_init(&config);

	init_eeprom(rom_data, FV1_PGM_SIZE);
	init_gpios();

	while (1) {
		/* Wait until we have received fresh data over serial */
		k_sem_take(&serial_data, K_FOREVER);
		load_program(serial_packet, sizeof(serial_packet));
	}
}
