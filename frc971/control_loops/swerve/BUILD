load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

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
