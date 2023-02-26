#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "serial.h"

LOG_MODULE_REGISTER(rx_uart, 1);

static inline void cleanup_state(struct rx_uart *config)
{
	LOG_DBG("");
	memset(config->header, 0, sizeof(struct rx_uart_header));
}

/* False if header is invalid or incomplete
 * True if header complete and valid
 */
static bool build_header(struct rx_uart *uart_config)
{
	struct rx_uart_header *header = uart_config->header;

	__ASSERT_NO_MSG(header != NULL);

	if (header->idx > 6) {
		/* Header is complete, the current byte doesn't belong to it */
		return true;
	}

	uint8_t byte;

	if (ring_buf_get(uart_config->ringbuf, &byte, 1) != 1) {
		return false;
	}

	if (header->idx < 3) {
		if (byte != "FV1"[header->idx]) {
			return false;
		}
		/* If the rx char matches its required value, header->idx will
		 * be incremented and parsing will continue in the next call.
		 * Else, we cleanup the state and return.
		 */
	} else if (header->idx == 2) {
		/* Don't trigger a memset for each rx'd byte (that doesn't match
		 * the header).
		 */
		cleanup_state(uart_config);
	}

	switch (header->idx) {
		case 3:
			header->opcode = byte;
			break;
		case 4:
			header->crc = byte;
			break;
		case 5:
			header->len = byte << 8;
			break;
		case 6:
			header->len += byte;
			break;
		default:
			break;
	}

	LOG_DBG("byte[%d]: %x", header->idx, byte);

	header->idx++;
	return false;
}

static uint8_t compute_crc(struct rx_uart_header *header, struct ring_buf *buf)
{
	/* TODO: implement crc. could be crc8_ccitt() */
	return header->crc;
}

static void process_ringbuf(struct rx_uart *uart_config)
{
	struct rx_uart_header *header = uart_config->header;

	/* try to parse header */
	while (!ring_buf_is_empty(uart_config->ringbuf) &&
	       !build_header(uart_config)) {};

	/* receive the packet data */
	if (build_header(uart_config)) {
		int left = ring_buf_size_get(uart_config->ringbuf) - header->len;
		if (left >= 0) {
			LOG_DBG("ringbuf has correct size");

			if (compute_crc(header, uart_config->ringbuf) == header->crc) {
				__ASSERT_NO_MSG(uart_config->cb);
				uart_config->cb(uart_config);
				cleanup_state(uart_config);

				/* Re-trigger processing in case we have another packet pending. */
				process_ringbuf(uart_config);

				return;
			}
		} else {
			LOG_DBG("missing %d bytes", left);
		}
	}
}

/*
 * Read any available characters from UART, and place them in a ring buffer. The
 * ring buffer is in turn processed by process_ringbuf().
 */
static void serial_cb(const struct device *uart, void *user_data)
{
	LOG_DBG("");
	struct rx_uart *uart_config = (struct rx_uart *)user_data;

	if (!uart_irq_update(uart)) {
		return;
	}

	while (uart_irq_rx_ready(uart)) {
		uint8_t byte = 0; /* Have to assign to stop GCC from whining */
		while(uart_fifo_read(uart, &byte, 1) > 0) {
			uint32_t ret = ring_buf_put(uart_config->ringbuf, &byte, 1);
			(void)ret; /* Don't warn if logging is disabled */

			LOG_DBG("rx: %x, rb put %u", byte, ret);

			process_ringbuf(uart_config);
		}
	}
}

int serial_init(struct rx_uart* uart_config)
{
	LOG_DBG("");

	/* Initialize UART driver */
	if (!device_is_ready(uart_config->uart)) {
		LOG_ERR("UART device not found!");
		return -EAGAIN;
	}

	uart_irq_callback_user_data_set(uart_config->uart, serial_cb, (void *)uart_config);
	uart_irq_rx_enable(uart_config->uart);

	LOG_DBG("init ok");

	return 0;
}
