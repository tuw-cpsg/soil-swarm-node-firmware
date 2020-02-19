cmake_minimum_required(VERSION 3.8.2)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD rfd77101_soil_swarm_node_hardware)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(peripheral)

target_sources(app PRIVATE
  src/main.c
  src/node.c
  src/util.c
  
  src/battery.c
  src/moisture.c
  src/onewire.c
  src/ds18b20.c
  
  src/ass.c
  src/bas.c
  src/cts.c
  src/ess.c
  
  # src/sys_fatal_error_handler.c
)
