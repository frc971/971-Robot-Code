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
    visibility = ["//visibility:public"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_can_position_fbs",
    srcs = ["swerve_drivetrain_can_position.fbs"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_position_fbs",
    srcs = ["swerve_drivetrain_position.fbs"],
    visibility = ["//visibility:public"],
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
