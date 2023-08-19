#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>

#include "view.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(input_app, 4);

static struct viewstate vs = {.program = 07,
.active = 1,
.values[0] = 10,
.values[1] = 20,
.values[2] = 30,
.names = {"par 0", "par 1", "par 2"}};

static void input_cb(struct input_event *evt)
{
	/* input_app: input_cb: sync 1 type 2 code 0x1 value 1
	 * -> value = -1 in opposite direction
	 */
	LOG_ERR("%s: sync %u type %u code 0x%x value %d", __func__,
		evt->sync, evt->type, evt->code, evt->value);

	vs.program += evt->value;
	if (vs.program > 99) vs.program = 99;
	if (vs.program < 0) vs.program = 0;

	draw_view(&vs);
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_PATH(encoder)), input_cb);
