#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "serial.h"
#include "dsp.h"

LOG_MODULE_REGISTER(rx_uart, 1);

struct rx_uart_header {
	char start[3];		/* spells F V 1 */
	uint8_t opcode;		/* Opcode of the packet */
	uint8_t crc;		/* CRC of whole frame */
	uint16_t len;
	uint8_t idx;		/* Current index, used when building header */
};

struct rx_uart {
	const struct device *uart;

	struct rx_uart_header *header;

	/* ring buffer: stores all received uart data */
	struct ring_buf *ringbuf;

	/* Called when a packet was successfully received */
	void (*cb)(struct rx_uart* config);
};

enum opcodes {
	OP_LOAD = 0,
	OP_POT,
};

RING_BUF_DECLARE(uart_ringbuf, 1024);
static struct rx_uart_header uart_header;

static void process_serial(struct rx_uart *config);
static struct rx_uart uart_config = {
	.uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0)),
	.header = &uart_header,
	.ringbuf = &uart_ringbuf,
	.cb = process_serial,
};

static inline void cleanup_state(struct rx_uart *config)
{
	LOG_DBG("");
	memset(config->header, 0, sizeof(struct rx_uart_header));
}

/* False if header is invalid or incomplete
 * True if header complete and valid
 */
static bool build_header(struct rx_uart *config)
{
	struct rx_uart_header *header = config->header;

	__ASSERT_NO_MSG(header != NULL);

	if (header->idx > 6) {
		/* Header is complete, the current byte doesn't belong to it */
		return true;
	}

	uint8_t byte;

	if (ring_buf_get(config->ringbuf, &byte, 1) != 1) {
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
		cleanup_state(config);
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

static void process_ringbuf(struct rx_uart *config)
{
	struct rx_uart_header *header = config->header;

	/* try to parse header */
	while (!ring_buf_is_empty(config->ringbuf) &&
	       !build_header(config)) {};

	/* receive the packet data */
	if (build_header(config)) {
		int left = ring_buf_size_get(config->ringbuf) - header->len;
		if (left >= 0) {
			LOG_DBG("ringbuf has correct size");

			if (compute_crc(header, config->ringbuf) == header->crc) {
				__ASSERT_NO_MSG(config->cb);
				config->cb(config);
				cleanup_state(config);

				/* Re-trigger processing in case we have another packet pending. */
				process_ringbuf(config);

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
	struct rx_uart *config = (struct rx_uart *)user_data;

	if (!uart_irq_update(uart)) {
		return;
	}

	while (uart_irq_rx_ready(uart)) {
		uint8_t byte = 0; /* Have to assign to stop GCC from whining */
		while(uart_fifo_read(uart, &byte, 1) > 0) {
			uint32_t ret = ring_buf_put(config->ringbuf, &byte, 1);
			(void)ret; /* Don't warn if logging is disabled */

			LOG_DBG("rx: %x, rb put %u", byte, ret);

			process_ringbuf(config);
		}
	}
}

int serial_init(void)
{
	LOG_DBG("");

	/* only 1 serial command interface */
	struct rx_uart *config = &uart_config;

	/* Initialize UART driver */
	if (!device_is_ready(config->uart)) {
		LOG_ERR("UART device not found!");
		return -EAGAIN;
	}

	uart_irq_callback_user_data_set(config->uart, serial_cb, (void *)config);
	uart_irq_rx_enable(config->uart);

	LOG_DBG("SERIAL init ok");

	return 0;
}

void print_bad_data(struct ring_buf *ringbuf, uint16_t len)
{
	/* get rid of this after development */
	static char tmp[1000] = {0};

	len = MIN(len, sizeof(tmp));
	ring_buf_get(ringbuf, tmp, len);
	LOG_HEXDUMP_WRN(tmp, sizeof(tmp), "unexpected opcode");
}

static void process_serial(struct rx_uart *config)
{
	switch (config->header->opcode) {
		case OP_LOAD:
			dsp_load_from_serial(config->ringbuf, config->header->len);
			break;
		case OP_POT:
			dsp_set_pot_values(config->ringbuf, config->header->len);
			break;
		default:
			LOG_ERR("unexpected opcode 0x%x", config->header->opcode);
			print_bad_data(config->ringbuf, config->header->len);
			break;
	}
}
