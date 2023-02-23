cc_binary(
    name = "imu_main",
    srcs = ["imu_main.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/imu_reader:imu",
        "//y2023:constants",
    ],
)
