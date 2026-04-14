cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(${CMAKE_CURRENT_LIST_DIR}/libmicroros)

add_executable(pico_micro_ros
  src/main.c
)

target_include_directories(pico_micro_ros PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

target_link_libraries(pico_micro_ros
  pico_stdlib
  pico_stdio_usb
  pico_cyw43_arch_none
  hardware_pwm
  hardware_i2c
  microros
)

pico_enable_stdio_usb(pico_micro_ros 1)
pico_enable_stdio_uart(pico_micro_ros 0)

pico_add_extra_outputs(pico_micro_ros)
