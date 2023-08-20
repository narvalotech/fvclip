cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD fvclip)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})

project(fvctl)

target_sources(app PRIVATE
  fonts/creep.c
  src/main.c
  src/dsp.c
  src/disp.c
  src/input.c
  src/serial.c
  src/eeprom.c)
