cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(fvctl)

target_sources(app PRIVATE
  src/main.c
  src/eeprom.c)
