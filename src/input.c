#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(input_app, 4);

static void input_cb(struct input_event *evt)
{
	LOG_ERR("%s: sync %u type %u code 0x%x value %d", __func__,
		evt->sync, evt->type, evt->code, evt->value);
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_PATH(encoder)), input_cb);
