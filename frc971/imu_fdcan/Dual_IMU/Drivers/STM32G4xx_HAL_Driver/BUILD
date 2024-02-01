cc_library(
    name = "hal_driver",
    srcs = glob([
        "Src/*.c",
    ]),
    hdrs = glob([
        "Inc/**/*.h",
    ]),
    includes = ["Inc"],
    visibility = ["//frc971/imu_fdcan:__subpackages__"],
    deps = [
        "//frc971/imu_fdcan/Dual_IMU/Core:config",
        "//frc971/imu_fdcan/Dual_IMU/Drivers/CMSIS:cmsis",
    ],
)
