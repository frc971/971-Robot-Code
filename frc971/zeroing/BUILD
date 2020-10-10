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
        "absolute_encoder.cc",
        "hall_effect_and_position.cc",
        "pot_and_absolute_encoder.cc",
        "pot_and_index.cc",
        "pulse_index.cc",
        "absolute_and_absolute_encoder.cc",
    ],
    hdrs = [
        "absolute_encoder.h",
        "hall_effect_and_position.h",
        "pot_and_absolute_encoder.h",
        "pot_and_index.h",
        "pulse_index.h",
        "zeroing.h",
        "absolute_and_absolute_encoder.h",
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
        "absolute_encoder_test.cc",
        "hall_effect_and_position_test.cc",
        "pot_and_absolute_encoder_test.cc",
        "pot_and_index_test.cc",
        "pulse_index_test.cc",
        "relative_encoder_test.cc",
        "absolute_and_absolute_encoder_test.cc",
        "zeroing_test.h",
    ],
    deps = [
        ":zeroing",
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
