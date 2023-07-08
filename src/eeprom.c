#include <zephyr/device.h>
#include <nrfx_twis.h>

#include "eeprom.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eeprom, 4);

struct fakerom {
	uint16_t addr;
	uint16_t size;
	uint8_t* data;
};

static struct fakerom rom;

/* Should match master's buffer size */
static unsigned char twis_rx_buffer[8];
const nrfx_twis_t twis_instance = NRFX_TWIS_INSTANCE(0);

void i2c_set_tx(uint8_t *buf, uint16_t len)
{
	LOG_DBG("set TX: buf %p len %u", (void*)buf, len);
	(void)nrfx_twis_tx_prepare(&twis_instance, buf, len);
}

static void twis_event_handler(nrfx_twis_evt_t const *const p_event)
{
	switch (p_event->type) {

	case NRFX_TWIS_EVT_READ_REQ:
		/* This case shouldn't be needed at all. It is only called if no
		 * buffers are ready when a read request comes in.
		 */
		if (p_event->data.buf_req) {
			/* When we get here, it's too late, FV1 will have failed to read the
			 * program and will just hang until the next transaction. */
			LOG_ERR("Ran out of TX data, reset addr to 0x00");
			i2c_set_tx(rom.data, rom.size);
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
		LOG_ERR("Unhandled TWI event");
		break;
	}
}

void init_i2c()
{
	nrfx_twis_config_t twis_config;

	memset(&twis_config, 0, sizeof(twis_config));

	twis_config.addr[0] = DT_REG_ADDR(DT_NODELABEL(eeprom));

	/* TODO: don't hardcode values, use pinctrl + DT macros */
	twis_config.scl_pin = 31;
	twis_config.scl_pull = NRF_GPIO_PIN_PULLUP;

	twis_config.sda_pin = 29;
	twis_config.sda_pull = NRF_GPIO_PIN_PULLUP;

	twis_config.interrupt_priority = DT_IRQ(DT_NODELABEL(i2c0), priority);

	LOG_DBG("I2C Slave: ADDR: 0x%x, SCL: %u, SDA: %u, int_pri: %u",
	       twis_config.addr[0], twis_config.scl_pin, twis_config.sda_pin, twis_config.interrupt_priority);

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
}

void init_eeprom(uint8_t *buf, uint16_t size)
{
	rom.addr = 0;
	rom.size = size;
	rom.data = buf;

	/* Make the next read request read from the first EEPROM address. */
	i2c_set_tx(rom.data, rom.size);

	LOG_INF("eeprom init ok");
}
