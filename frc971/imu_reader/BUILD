load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

cc_library(
    name = "imu",
    srcs = [
        "imu.cc",
    ],
    hdrs = [
        "imu.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:epoll",
        "//aos/events:shm_event_loop",
        "//aos/util:crc32",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

flatbuffer_cc_library(
    name = "imu_failures_fbs",
    srcs = [
        "imu_failures.fbs",
    ],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "imu_failures_ts_fbs",
    srcs = [
        "imu_failures.fbs",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "imu_watcher",
    srcs = ["imu_watcher.cc"],
    hdrs = ["imu_watcher.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":imu_failures_fbs",
        "//aos/events:event_loop",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/zeroing:imu_zeroer",
        "//frc971/zeroing:wrap",
    ],
)
