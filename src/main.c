#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#include "eeprom.h"

/* Built-in programs */
#include "rom.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, 4);

#define EEPROM_SIZE 4096
static uint8_t rom_data[EEPROM_SIZE];

void make_fake_data(uint8_t * data, size_t bytes)
{
	for (int a=0; a<bytes; a++) {
		data[a] = a & 0xFF;
	}
}

#define GPIO_S0_PIN DT_PHA(DT_PATH(outputs, gpio_s0), gpios, pin)
#define GPIO_S1_PIN DT_PHA(DT_PATH(outputs, gpio_s1), gpios, pin)
#define GPIO_S2_PIN DT_PHA(DT_PATH(outputs, gpio_s2), gpios, pin)
#define GPIO_EXT_PIN DT_PHA(DT_PATH(outputs, gpio_ext), gpios, pin)
#define GPIO_CLIP_PIN DT_PHA(DT_PATH(inputs, gpio_clip), gpios, pin)

void init_gpios(void)
{
	const struct device *port =
		DEVICE_DT_GET(DT_PHANDLE(DT_PATH(outputs, gpio_s0), gpios));

	uint32_t out_flags = GPIO_OUTPUT;
	uint32_t in_flags = GPIO_INPUT;

	gpio_pin_configure(port, GPIO_S0_PIN, out_flags);
	gpio_pin_configure(port, GPIO_S1_PIN, out_flags);
	gpio_pin_configure(port, GPIO_S2_PIN, out_flags);
	gpio_pin_configure(port, GPIO_EXT_PIN, out_flags);

	gpio_pin_configure(port, GPIO_CLIP_PIN, in_flags);
}

void select_program(uint8_t id)
{
	const struct device *port =
		DEVICE_DT_GET(DT_PHANDLE(DT_PATH(outputs, gpio_s0), gpios));

	if (id > 7) {
		LOG_ERR("Program ID out of bounds (> 7)");
		return;
	}

	LOG_INF("select program %u", id);
	gpio_pin_set(port, GPIO_S0_PIN, id & 1);
	gpio_pin_set(port, GPIO_S1_PIN, id & 2);
	gpio_pin_set(port, GPIO_S2_PIN, id & 4);
}

enum {
	PGM_BUILTIN = 0,
	PGM_EXTERNAL,
};

void select_program_source(bool ext)
{
	const struct device *port =
		DEVICE_DT_GET(DT_PHANDLE(DT_PATH(outputs, gpio_s0), gpios));

	gpio_pin_set(port, GPIO_EXT_PIN, ext);
}

void load_program(uint8_t id)
{
	set_offset_eeprom(id * 512);
}

void load_rom(uint8_t * data, size_t bytes)
{
	/* Only copy the very first program */
	memcpy(data, samples_00, sizeof(samples_00));
	memcpy(data + 512, samples_04, sizeof(samples_04));
	LOG_INF("Loaded %u bytes into fake-ROM", 1024);
}

void main(void)
{
	LOG_ERR("Bootup");

	/* make_fake_data(rom_data, EEPROM_SIZE); */
	load_rom(rom_data, EEPROM_SIZE);

	init_eeprom(rom_data, EEPROM_SIZE);
	init_gpios();

	/* select_program_source(PGM_BUILTIN); */
	select_program_source(PGM_EXTERNAL);
	select_program(0);

	while (1)
		for (int id=0; id < 2; id++) {
			load_program(id);
			select_program(id);
			k_sleep(K_SECONDS(10));
		}
}
