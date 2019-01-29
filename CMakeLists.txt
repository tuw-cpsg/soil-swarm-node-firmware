cmake_minimum_required(VERSION 3.8.2)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(peripheral)

target_sources(app PRIVATE
  src/main.c
  src/bas.c
  src/ess.c
  src/led.c
  src/battery.c
  src/moisture.c
  src/onewire.c
  src/ds18b20.c
  src/power.c
  src/device.c
)

zephyr_library_include_directories($ENV{ZEPHYR_BASE}/samples/bluetooth)
