#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/display/cfb.h>
#include <stdio.h>

#include "view.h"
#include "utils.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(disp, 4);

void view_init(void)
{
	const struct device *dev =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(dev)) {
		LOG_ERR("Device %s not ready", dev->name);
		k_oops();
	}

	if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO10) != 0) {
		LOG_ERR("Failed to set required pixel format");
		k_oops();
	}

	if (cfb_framebuffer_init(dev)) {
		LOG_ERR("Framebuffer initialization failed!");
		k_oops();
	}

	display_blanking_off(dev);

	printk("VIEW INIT OK\n");
}

enum {
	FONT_SMALL = 0,
	FONT_MEDIUM,
	FONT_LARGE
};

void draw_view(struct viewstate *state)
{
	LOG_DBG("");
	char str[20];
	const struct device *dev =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	cfb_framebuffer_clear(dev, false);

	/* Draw program ID */
	cfb_framebuffer_set_font(dev, FONT_LARGE);
	sprintf(str, "%02d", state->program);
	cfb_print(dev, str, 0, 0);

	/* Draw params */
	cfb_framebuffer_set_font(dev, FONT_SMALL);
	for (int i=0; i<3; i++) {
		sprintf(str, "%s: %02u", &state->names[i][0], state->values[i]);
		cfb_draw_text(dev, str, 50, i * 10);
	}

	/* TODO: invert active */

	/* STATUS:
	 * - aaaalmost fits: need smaller (bitmap) font for params
	 */

	/* Flush buffer to display */
	cfb_framebuffer_finalize(dev);

	LOG_DBG("updated");
}

static int enable_disp(void)
{
	/* Power-cycle the display and its driver */
	gpio_pin_configure(PORT_PIN(outputs, gpio_oled_pwr), GPIO_OUTPUT);
	gpio_pin_set(PORT_PIN(outputs, gpio_oled_pwr), 1);

	k_msleep(50);

	gpio_pin_configure(PORT_PIN(outputs, gpio_oled_pwr), GPIO_OUTPUT);
	gpio_pin_set(PORT_PIN(outputs, gpio_oled_pwr), 0);

	k_msleep(50);

	return 0;
}

/* power the display before driver inits */
#define DISPLAY_POWER_INIT_PRIORITY 70

BUILD_ASSERT(CONFIG_DISPLAY_INIT_PRIORITY > DISPLAY_POWER_INIT_PRIORITY);

/* would the `reset` dts property do the trick? */
SYS_INIT(enable_disp, POST_KERNEL, DISPLAY_POWER_INIT_PRIORITY);
