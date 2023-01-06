#ifndef SERIAL_H_
#define SERIAL_H_

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
	void (*cb)(struct ring_buf *ringbuf, uint16_t len);
};

int serial_init(struct rx_uart* uart_config);

#endif /* SERIAL_H_ */
