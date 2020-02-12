package(default_visibility = ["//visibility:public"])

load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "averager",
    hdrs = [
        "averager.h",
    ],
    deps = [
        "@com_github_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "averager_test",
    srcs = [
        "averager_test.cc",
    ],
    deps = [
        ":averager",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "imu_zeroer",
    srcs = [
        "imu_zeroer.cc",
    ],
    hdrs = [
        "imu_zeroer.h",
    ],
    deps = [
        ":averager",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/wpilib:imu_fbs",
        "@com_github_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "imu_zeroer_test",
    srcs = [
        "imu_zeroer_test.cc",
    ],
    deps = [
        ":imu_zeroer",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "zeroing",
    srcs = [
        "zeroing.cc",
    ],
    hdrs = [
        "zeroing.h",
    ],
    deps = [
        ":wrap",
        "//aos/logging",
        "//frc971:constants",
        "//frc971/control_loops:control_loops_fbs",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "zeroing_test",
    srcs = [
        "zeroing_test.cc",
    ],
    deps = [
        ":zeroing",
        "//aos:die",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:position_sensor_sim",
    ],
)

cc_library(
    name = "wrap",
    srcs = [
        "wrap.cc",
    ],
    hdrs = [
        "wrap.h",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "wrap_test",
    srcs = [
        "wrap_test.cc",
    ],
    deps = [
        ":wrap",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "unwrap_test",
    srcs = [
        "unwrap_test.cc",
    ],
    deps = [
        ":wrap",
        "//aos/testing:googletest",
    ],
)
