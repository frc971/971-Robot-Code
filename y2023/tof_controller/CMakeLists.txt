
cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()

#set(PICO_SDK_FETCH_FROM_GIT 1)
set(PICO_SDK_PATH ../../../third_party/pico-sdk)
set(PICO_TOOLCHAIN_PATH ../../external/gcc_arm_none_eabi/bin)

include(../../third_party/pico-sdk/external/pico_sdk_import.cmake)

project(tof_controller)

# the root of the repository
include_directories(../../)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(CMAKE_CXX_STANDARD 17)

# rest of your project

include_directories(../../external/vl53l1x_ultra_lite_driver_api)
add_library(VL53L1X
        ../../external/vl53l1x_ultra_lite_driver_api/platform/vl53l1_platform.h
        ../../external/vl53l1x_ultra_lite_driver_api/platform/vl53l1_platform.c
        ../../external/vl53l1x_ultra_lite_driver_api/platform/vl53l1_types.h
        ../../external/vl53l1x_ultra_lite_driver_api/core/VL53L1X_api.c
        ../../external/vl53l1x_ultra_lite_driver_api/core/VL53L1X_api.h
        ../../external/vl53l1x_ultra_lite_driver_api/core/VL53L1X_calibration.h
        ../../external/vl53l1x_ultra_lite_driver_api/core/VL53L1X_calibration.c)

add_executable(tof_controller
				tof_controller.cc
				)

# pull in common dependencies
target_link_libraries(tof_controller VL53L1X)
target_link_libraries(VL53L1X pico_stdlib hardware_i2c hardware_pwm)
target_link_libraries(tof_controller pico_stdlib hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(tof_controller 1)
pico_enable_stdio_uart(tof_controller 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(tof_controller)

