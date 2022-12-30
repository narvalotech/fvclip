#include <zephyr/device.h>
#include <nrfx_twis.h>

#include "eeprom.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eeprom, 4);

static uint16_t rom_addr;	/* current address pointer */
static uint8_t *rom_data;
static uint16_t rom_size;

/* TODO: move I2C driver to own file */
#define TWI_BUFFER_SIZE 8	/* Should match master's buffer size */
static unsigned char twis_rx_buffer[TWI_BUFFER_SIZE];

const nrfx_twis_t twis_instance = NRFX_TWIS_INSTANCE(0);

void i2c_set_tx(uint8_t *buf, uint16_t len)
{
	LOG_DBG("set TX: buf %p len %u", (void*)buf, len);

	(void)nrfx_twis_tx_prepare(&twis_instance,
				   &rom_data[rom_addr],
				   rom_size - rom_addr);
}

/* Called when master initiates a read transaction and the I2C buffer is not
 * configured in the peripheral.
 */
void i2c_tx_cb(uint8_t **buf, uint16_t *len)
{
	*buf = &rom_data[0];	/* TODO: use current address */
	*len = rom_size;	/* TODO: handle rollover */
}

/* Called when master finishes a write transaction */
void i2c_rx_cb(uint8_t *buf, uint16_t len)
{
	/* We should only ever receive 2 bytes: addr MSB and LSB */
	LOG_HEXDUMP_DBG(buf, len, "I2C RX:");

	if (len > 2) {
		LOG_ERR("Got more than 2 bytes write, ignoring..");
		return;
	}

	__ASSERT_NO_MSG(len == 2);
	rom_addr = buf[0] << 8;
	rom_addr |= buf[1];
	rom_addr &= 0xFFF;	/* addr is only 12 bits */

	LOG_INF("Changing address to 0x%x", rom_addr);

	i2c_set_tx(&rom_data[rom_addr], rom_size - rom_addr);
}

static void twis_event_handler(nrfx_twis_evt_t const *const p_event)
{
	switch (p_event->type) {

	case NRFX_TWIS_EVT_READ_REQ:
		/* This case shouldn't be needed at all. It is only called if no
		 * buffers are ready when a read request comes in.
		 */
		if (p_event->data.buf_req) {
			uint16_t len;
			uint8_t *buf;

			i2c_tx_cb(&buf, &len);
			(void)nrfx_twis_tx_prepare(&twis_instance, buf, len);
		}
		break;

	case NRFX_TWIS_EVT_READ_DONE:
		LOG_DBG("TWIS READ done (%u bytes)", p_event->data.tx_amount);
		break;

	case NRFX_TWIS_EVT_WRITE_REQ:
		/* Ditto with READ_REQ. */
		if (p_event->data.buf_req) {
			(void)nrfx_twis_rx_prepare(&twis_instance,
						   twis_rx_buffer,
						   sizeof(twis_rx_buffer));
		}
		break;

	case NRFX_TWIS_EVT_WRITE_DONE:
		i2c_rx_cb(twis_rx_buffer, p_event->data.rx_amount);
		LOG_DBG("TWIS WRITE done (%u bytes)", p_event->data.rx_amount);
		break;

	case NRFX_TWIS_EVT_READ_ERROR:
		LOG_DBG("TWIS READ ERROR");
		break;

	case NRFX_TWIS_EVT_WRITE_ERROR:
		LOG_DBG("TWIS WRITE ERROR");
		break;

	case NRFX_TWIS_EVT_GENERAL_ERROR:
		LOG_DBG("TWIS GENERAL ERROR");
		break;

	default:
		LOG_DBG("TWIS default");
		break;
	}
}

void init_eeprom(uint8_t *buf, uint16_t size)
{
	rom_size = size;
	nrfx_twis_config_t twis_config;

	memset(&twis_config, 0, sizeof(twis_config));

	twis_config.addr[0] = DT_REG_ADDR(DT_NODELABEL(eeprom));

	/* TODO: move to pinctrl */
	twis_config.scl = DT_PROP(DT_NODELABEL(i2c0), scl_pin);
	twis_config.scl_pull = NRF_GPIO_PIN_PULLUP;

	twis_config.sda = DT_PROP(DT_NODELABEL(i2c0), sda_pin);
	twis_config.sda_pull = NRF_GPIO_PIN_PULLUP;

	twis_config.interrupt_priority = DT_IRQ(DT_NODELABEL(i2c0), priority);

	LOG_DBG("I2C Slave: ADDR: 0x%x, SCL: %u, SDA: %u, int_pri: %u\n",
	       twis_config.addr[0], twis_config.scl, twis_config.sda, twis_config.interrupt_priority);

	if (nrfx_twis_init(&twis_instance, &twis_config, twis_event_handler) == NRFX_SUCCESS) {
		LOG_DBG("nrfx TWIS initialized.");
	} else {
		LOG_DBG("failed initializing TWIS");
	}

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2c0)),
		    DT_IRQ(DT_NODELABEL(i2c0), priority),
		    nrfx_isr, nrfx_twis_0_irq_handler, 0);

	nrfx_twis_enable(&twis_instance);

	/* RX buffer always points to the same thing. */
	(void)nrfx_twis_rx_prepare(&twis_instance, twis_rx_buffer, sizeof(twis_rx_buffer));
	LOG_DBG("eeprom init ok");

	/* Make the next read request read from the first EEPROM address. */
	(void)nrfx_twis_tx_prepare(&twis_instance, rom_data, rom_size);

	/* Reset address pointer */
	rom_addr = 0;
}
