load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "localizer_plotter",
    srcs = ["localizer_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//frc971/wpilib:imu_plot_utils",
    ],
)

flatbuffer_cc_library(
    name = "localizer_output_fbs",
    srcs = [
        "localizer_output.fbs",
    ],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "localizer_status_fbs",
    srcs = [
        "localizer_status.fbs",
    ],
    gen_reflections = True,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_static_flatbuffer(
    name = "localizer_schema",
    function = "frc971::controls::LocalizerStatusSchema",
    target = ":localizer_status_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "localizer",
    srcs = ["localizer.cc"],
    hdrs = ["localizer.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":localizer_output_fbs",
        ":localizer_status_fbs",
        "//aos/containers:ring_buffer",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/time",
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/control_loops/drivetrain:localizer_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//frc971/zeroing:imu_zeroer",
        "//frc971/zeroing:wrap",
        "//y2022:constants",
        "//y2022/control_loops/superstructure:superstructure_status_fbs",
        "//y2022/vision:calibration_fbs",
        "//y2022/vision:target_estimate_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "localizer_main",
    srcs = ["localizer_main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":localizer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//y2022/control_loops/drivetrain:drivetrain_base",
    ],
)

cc_test(
    name = "localizer_test",
    srcs = ["localizer_test.cc"],
    data = [
        "//y2022:aos_config",
    ],
    shard_count = 13,
    deps = [
        ":localizer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//frc971/control_loops/drivetrain:drivetrain_test_lib",
        "//y2022/control_loops/drivetrain:drivetrain_base",
    ],
)

cc_binary(
    name = "localizer_replay",
    srcs = ["localizer_replay.cc"],
    data = [
        "//y2022:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":localizer",
        ":localizer_schema",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//y2022/control_loops/drivetrain:drivetrain_base",
    ],
)

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
