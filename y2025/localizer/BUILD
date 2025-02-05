load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localizer",
    srcs = [
        "kalman_filter_localizer.cc",
        "localizer.cc",
        "weighted_average_localizer.cc",
    ],
    hdrs = ["localizer.h"],
    deps = [
        ":status_fbs",
        "//aos/time",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/drivetrain/localization:utils",
        "//frc971/control_loops/swerve:autonomous_init_fbs",
        "//frc971/control_loops/swerve:simplified_dynamics",
        "//frc971/control_loops/swerve:swerve_drivetrain_status_fbs",
        "//frc971/control_loops/swerve:swerve_localizer_state_fbs",
        "//frc971/estimation:kf",
        "//frc971/vision:target_map_fbs",
        "//frc971/vision:target_map_utils",
        "//y2025/constants:constants_fbs",
    ],
)

cc_test(
    name = "localizer_test",
    srcs = ["localizer_test.cc"],
    data = [
        "//y2025:aos_config",
        "//y2025/constants:test_constants_json",
    ],
    deps = [
        ":localizer",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//y2025/constants:simulated_constants_sender",
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
        "//frc971/constants:constants_sender_lib",
    ],
)

cc_binary(
    name = "localizer_replay",
    srcs = ["localizer_replay.cc"],
    data = [
        "//y2025:aos_config",
        "//y2025/constants:constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":localizer",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/util:simulation_logger",
        "//frc971/constants:constants_sender_lib",
        "//frc971/imu_fdcan:dual_imu_blender_lib",
        "//y2025/constants:simulated_constants_sender",
    ],
)

static_flatbuffer(
    name = "status_fbs",
    srcs = ["status.fbs"],
)
