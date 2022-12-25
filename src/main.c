#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>

#include <nrfx_twis.h>

#define EEPROM_SIZE 1024
static uint8_t rom_data[EEPROM_SIZE];

#define CHECK(x) do {					\
		int err;				\
		err = x;				\
		__ASSERT(err == 0, "errcode: %d", err);	\
	} while(0)

void make_fake_data(uint8_t * data, size_t bytes)
{
	for (int a=0; a<bytes; a++) {
		data[a] = a & 0xFF;
	}
}

/* Called when master initiates a read transaction */
void i2c_tx_cb(uint8_t **buf, uint16_t *len)
{
	*buf = &rom_data[0];	/* TODO: use current address */
	*len = EEPROM_SIZE;	/* TODO: handle rollover */
}

/* TODO: move I2C driver to own file */
#define TWI_BUFFER_SIZE 8	/* Should match master's buffer size */
static unsigned char twis_rx_buffer[TWI_BUFFER_SIZE];

const nrfx_twis_t twis_instance = NRFX_TWIS_INSTANCE(0);

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
		printk("TWIS READ done\n");
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
		printk("TWIS WRITE done\n");
		break;

	case NRFX_TWIS_EVT_READ_ERROR:
		printk("TWIS READ ERROR\n");
		break;

	case NRFX_TWIS_EVT_WRITE_ERROR:
		printk("TWIS WRITE ERROR\n");
		break;

	case NRFX_TWIS_EVT_GENERAL_ERROR:
		printk("TWIS GENERAL ERROR\n");
		break;

	default:
		printk("TWIS default\n");
		break;
	}
}

void twis_init(void)
{
	nrfx_twis_config_t twis_config;

	memset(&twis_config, 0, sizeof(twis_config));

	twis_config.addr[0] = DT_REG_ADDR(DT_NODELABEL(eeprom));

	/* TODO: move to pinctrl */
	twis_config.scl = DT_PROP(DT_NODELABEL(i2c0), scl_pin);
	twis_config.scl_pull = NRF_GPIO_PIN_PULLUP;

	twis_config.sda = DT_PROP(DT_NODELABEL(i2c0), sda_pin);
	twis_config.sda_pull = NRF_GPIO_PIN_PULLUP;

	twis_config.interrupt_priority = DT_IRQ(DT_NODELABEL(i2c0), priority);

	printk("I2C Slave:\n ADDR: 0x%x, SCL: %u, SDA: %u, int_pri: %u\n",
	       twis_config.addr[0], twis_config.scl, twis_config.sda, twis_config.interrupt_priority);

	if (nrfx_twis_init(&twis_instance, &twis_config, twis_event_handler) == NRFX_SUCCESS) {
		printk("nrfx TWIS initialized.\n");
	} else {
		printk("failed initializing TWIS\n");
	}

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2c0)),
		    DT_IRQ(DT_NODELABEL(i2c0), priority),
		    nrfx_isr, nrfx_twis_0_irq_handler, 0);
}

void main(void)
{
	make_fake_data(rom_data, EEPROM_SIZE);

	twis_init();
	nrfx_twis_enable(&twis_instance);

	printk("main done\n");
	return;
}
