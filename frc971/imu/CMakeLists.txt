cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(ADIS16505_communication C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_executable(pico_imu_board)

pico_generate_pio_header(pico_imu_board ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(pico_imu_board PRIVATE ADIS16505.cc)

# pull in common dependencies
target_link_libraries(pico_imu_board
  pico_stdlib
  hardware_spi
  hardware_dma
  hardware_irq
  hardware_pwm
  hardware_pio
  hardware_timer
  pico_bootrom
  pico_double
  )

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_imu_board 1)
pico_enable_stdio_uart(pico_imu_board 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_imu_board)
