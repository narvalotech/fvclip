#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
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
LOG_MODULE_REGISTER(main, 3);

static struct viewstate vs = {.program = 07,
.active = 1,
.values[0] = 10,
.values[1] = 20,
.values[2] = 30,
.names = {"par 0", "par 1", "par 2"}};

static void init_pa(void)
{
	uint32_t out_flags = GPIO_OUTPUT;

	/* enable output audio driver */
	gpio_pin_configure(PORT_PIN(outputs, gpio_pa_pwr), out_flags);
	gpio_pin_set(PORT_PIN(outputs, gpio_pa_pwr), 0);
}

static void next_program(int increment)
{
	vs.program += increment;
	if (vs.program > 99) vs.program = 99;
	if (vs.program < 0) vs.program = 0;
}

static void next_param(void)
{
	vs.active++;
	/* overflow's a good thing, right? */
	if (vs.active > 2) vs.active = 0xFF;
}

static void tune_param(int increment)
{
	if (vs.active == 0xFF) return; /* no active param */

	int8_t *param = &vs.values[vs.active];

	*param += increment;
	if (*param > 99) *param = 99;
	if (*param < 0) *param = 0;
}

static void input_cb(struct input_event *evt)
{
	LOG_DBG("%s: sync %u type %u code 0x%x value %d", __func__,
		evt->sync, evt->type, evt->code, evt->value);

	/* encoder */
	if (evt->code == INPUT_REL_Y) {
		tune_param(evt->value);
	}

	/* right */
	if (evt->code == INPUT_KEY_0 && evt->value) {
		next_program(+1);
	}

	/* left */
	if (evt->code == INPUT_KEY_2 && evt->value) {
		next_program(-1);
	}

	/* mid */
	if (evt->code == INPUT_KEY_1 && evt->value) {
		next_param();
	}

	draw_view(&vs);
}

/* Invoke callback for all input devices */
INPUT_CALLBACK_DEFINE(NULL, input_cb);

void main(void)
{
	if (usb_enable(NULL)) {
		LOG_ERR("failed to enable USB");
		return;
	}

	LOG_INF("Bootup");

	init_dsp();
	init_pa();
	serial_init();

	view_init();
	draw_view(&vs);
}
