#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <stdio.h>

#include "view.h"

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

	if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO01) != 0) {
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

	const struct device *dev =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	cfb_framebuffer_clear(dev, false);

	/* Draw program ID */
	cfb_framebuffer_set_font(dev, FONT_LARGE);
	cfb_print(dev, "42", 0, 0);

	/* Draw param 1 */
	cfb_framebuffer_set_font(dev, FONT_SMALL);
	cfb_draw_text(dev, "param 0: 99", 50, 0);
	cfb_draw_text(dev, "param 1: 10", 50, 10);
	cfb_draw_text(dev, "param 2: 32", 50, 20);
	/* TODO: invert active */

	/* STATUS:
	 * - aaaalmost fits: need smaller (bitmap) font for params
	 * - display is inverted. should be flipped 180 degrees
	 */

	/* Flush buffer to display */
	cfb_framebuffer_finalize(dev);

	LOG_DBG("updated");
}
