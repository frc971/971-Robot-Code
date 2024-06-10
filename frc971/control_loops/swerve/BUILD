load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "swerve_drivetrain_goal_fbs",
    srcs = ["swerve_drivetrain_goal.fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_status_fbs",
    srcs = ["swerve_drivetrain_status.fbs"],
    deps = ["//frc971/control_loops:profiled_subsystem_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_output_fbs",
    srcs = ["swerve_drivetrain_output.fbs"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_can_position_fbs",
    srcs = ["swerve_drivetrain_can_position.fbs"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_position_fbs",
    srcs = ["swerve_drivetrain_position.fbs"],
    deps = ["//frc971/control_loops:control_loops_fbs"],
)

py_binary(
    name = "simulation",
    srcs = [
        "simulation.py",
    ],
    deps = [
        "//frc971/control_loops/python:controls",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//sympy",
    ],
)

cc_library(
    name = "swerve_control_loops",
    srcs = ["swerve_control_loops.cc"],
    hdrs = ["swerve_control_loops.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_output_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_status_fbs",
        "//frc971/control_loops:control_loop",
    ],
)

cc_test(
    name = "swerve_control_test",
    srcs = ["swerve_control_test.cc"],
    data = [
        ":aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":swerve_control_loops",
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_output_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_status_fbs",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

aos_config(
    name = "aos_config",
    src = "swerve.json",
    flatbuffers = [
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_output_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//frc971/input:aos_config"],
)
