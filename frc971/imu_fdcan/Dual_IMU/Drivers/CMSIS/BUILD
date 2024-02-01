cc_library(
    name = "cmsis",
    hdrs = glob([
        "**/*.h",
    ]),
    defines = [
        "STM32G473xx",
        "STM32G4",
        "USE_HAL_DRIVER",
    ],
    includes = [
        "Device/ST/STM32G4xx/Include",
        "Include",
    ],
    visibility = ["//frc971/imu_fdcan:__subpackages__"],
    deps = ["//frc971/imu_fdcan/Dual_IMU/Core:config"],
)
