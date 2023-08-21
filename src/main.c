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

	serial_init();
}
