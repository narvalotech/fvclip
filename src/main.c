#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <stdio.h>

#include "eeprom.h"

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

void main(void)
{
	make_fake_data(rom_data, EEPROM_SIZE);
	init_eeprom(rom_data, EEPROM_SIZE);
	init_gpios();

	select_program_source(PGM_BUILTIN);
	select_program(0);

	while (1)
		for (int id=0; id < 7; id++) {
			k_sleep(K_SECONDS(5));
			select_program(id);
		}
}
