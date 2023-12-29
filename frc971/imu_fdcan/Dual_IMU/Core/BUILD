load("//motors:macros.bzl", "hex_from_elf")
load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")

copy_file(
    name = "copy_startup",
    src = "Startup/startup_stm32g473retx.s",
    # Path of the output file, relative to this package.
    out = "Startup/startup_stm32g473retx.S",
)

cc_library(
    name = "config",
    hdrs = ["Inc/stm32g4xx_hal_conf.h"],
    includes = ["Inc"],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex-m4f-imu",
    ],
    visibility = ["//frc971/imu_fdcan:__subpackages__"],
)

cc_library(
    name = "startup",
    srcs = ["Startup/startup_stm32g473retx.S"],
    copts = [
        "-x assembler-with-cpp",
        "-v",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex-m4f-imu",
    ],
)

cc_binary(
    name = "main.elf",
    srcs = glob([
        "Src/*.c",
        "Inc/*.h",
    ]),
    copts = [
        "-Wno-unused-variable",
        "-Wno-unused-but-set-variable",
    ],
    includes = ["Inc"],
    deps = [
        ":startup",
        "//frc971/imu_fdcan/Dual_IMU/Drivers/STM32G4xx_HAL_Driver:hal_driver",
    ],
)

hex_from_elf(
    name = "main",
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex-m4f-imu",
    ],
)
