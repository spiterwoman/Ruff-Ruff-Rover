cmake_minimum_required(VERSION 3.13)
if(NOT DEFINED PICO_BOARD)
  # Default to RP2040 Pico; override with -DPICO_BOARD=<board> when needed.
  set(PICO_BOARD pico)
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros C CXX ASM)
option(ROVER_PICO_DIAG "Build minimal micro-ROS diagnostic firmware" OFF)
option(ROVER_PICO_USB_TX_TEST "Build pure USB TX test firmware" OFF)
option(ROVER_PICO_TRANSPORT_TX_TEST "Build direct custom transport TX test firmware" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(${CMAKE_CURRENT_LIST_DIR}/libmicroros)

set(PICO_MICROROS_SOURCES pico_uart_transport.c)

if(ROVER_PICO_USB_TX_TEST)
  list(APPEND PICO_MICROROS_SOURCES src/main_usb_tx_test.c)
  add_compile_definitions(USBD_PRODUCT=\"RRR_PICO_USB_TX\")
elseif(ROVER_PICO_TRANSPORT_TX_TEST)
  list(APPEND PICO_MICROROS_SOURCES src/main_transport_tx_test.c)
  add_compile_definitions(USBD_PRODUCT=\"RRR_PICO_TRANSPORT_TX\")
elseif(ROVER_PICO_DIAG)
  list(APPEND PICO_MICROROS_SOURCES src/main_diag.c)
  add_compile_definitions(USBD_PRODUCT=\"RRR_PICO_DIAG\")
else()
  list(APPEND PICO_MICROROS_SOURCES src/main.c)
endif()

add_executable(pico_micro_ros ${PICO_MICROROS_SOURCES})

target_include_directories(pico_micro_ros PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

target_link_libraries(pico_micro_ros
  pico_stdlib
  hardware_pwm
  hardware_i2c
  microros
)

# CYW43 Wi-Fi stack exists on Pico W variants only.
if(TARGET pico_cyw43_arch_none)
  target_link_libraries(pico_micro_ros pico_cyw43_arch_none)
endif()

pico_enable_stdio_usb(pico_micro_ros 1)
pico_enable_stdio_uart(pico_micro_ros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
# Allow CDC stdio TX/RX even when host doesn't assert DTR.
add_compile_definitions(PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)

pico_add_extra_outputs(pico_micro_ros)
