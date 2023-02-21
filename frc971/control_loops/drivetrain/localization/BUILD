load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc971/input:joystick_state_fbs",
        "//frc971/vision:calibration_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "puppet_localizer",
    srcs = ["puppet_localizer.cc"],
    hdrs = ["puppet_localizer.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//frc971/control_loops/drivetrain:hybrid_ekf",
        "//frc971/control_loops/drivetrain:localizer",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
    ],
)

cc_test(
    name = "puppet_localizer_test",
    srcs = ["puppet_localizer_test.cc"],
    data = ["//y2022/control_loops/drivetrain:simulation_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":puppet_localizer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/network:team_number",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:drivetrain_test_lib",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//y2022/control_loops/drivetrain:drivetrain_base",
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

flatbuffer_ts_library(
    name = "localizer_output_ts_fbs",
    srcs = ["localizer_output.fbs"],
    visibility = ["//visibility:public"],
)
