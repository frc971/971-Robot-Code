cc_library(
    name = "imu",
    srcs = [
        "imu.cc",
    ],
    hdrs = [
        "imu.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:epoll",
        "//aos/events:shm_event_loop",
        "//aos/util:crc32",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//y2022:constants",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_binary(
    name = "imu_main",
    srcs = ["imu_main.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":imu",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)