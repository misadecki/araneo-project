cmake_minimum_required(VERSION 3.28.0)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/rpi_pico2.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(araneo LANGUAGES C)

target_include_directories(app PRIVATE inc)
target_sources(app PRIVATE
  src/main.c
  src/oled.c
  src/icons.c
  src/servo.c
)
