#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_device.h>

#include "dsp.h"
#include "eeprom.h"
#include "serial.h"
#include "utils.h"
#include "view.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, 4);

static void init_pa(void)
{
	uint32_t out_flags = GPIO_OUTPUT;

	/* enable output audio driver */
	gpio_pin_configure(PORT_PIN(outputs, gpio_pa_pwr), out_flags);
	gpio_pin_set(PORT_PIN(outputs, gpio_pa_pwr), 0);
}

RING_BUF_DECLARE(uart_ringbuf, 1024);
K_SEM_DEFINE(serial_data, 0, 1);
#define FV1_PGM_SIZE 512

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
	k_sem_give(&serial_data);
}

static struct rx_uart_header uart_header;
static struct rx_uart config = {
	.uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0)),
	.header = &uart_header,
	.ringbuf = &uart_ringbuf,
	.cb = process_serial,
};

void main(void)
{
	if (usb_enable(NULL)) {
		LOG_ERR("failed to enable USB");
		return;
	}

	LOG_ERR("Bootup");

	init_dsp();
	init_pa();

	/* test out the display */
	struct viewstate vs = {.program = 07,
			       .active = 1,
			       .values[0] = 10,
			       .values[1] = 20,
			       .values[2] = 30,
			       .names = {"param 0", "param 1", "param 2"}};

	view_init();
	draw_view(&vs);

	serial_init(&config);
}
