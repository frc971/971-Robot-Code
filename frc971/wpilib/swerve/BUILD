load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "swerve_drivetrain_writer",
    srcs = [
        "swerve_drivetrain_writer.cc",
    ],
    hdrs = [
        "swerve_drivetrain_writer.h",
    ],
    deps = [
        ":swerve_module",
        "//aos/logging",
        "//frc971:can_configuration_fbs",
        "//frc971/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc971/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc971/control_loops/swerve:swerve_drivetrain_position_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:talonfx",
        "//third_party:phoenix6",
    ],
)

cc_library(
    name = "swerve_module",
    hdrs = [
        "swerve_module.h",
    ],
    deps = [
        "//frc971/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc971/wpilib:talonfx",
    ],
)

cc_library(
    name = "swerve_util",
    hdrs = [
        "swerve_util.h",
    ],
    deps = [
        ":swerve_constants_fbs",
        "//frc971/wpilib:encoder_and_potentiometer",
    ],
)

static_flatbuffer(
    name = "swerve_constants_fbs",
    srcs = ["swerve_constants.fbs"],
    visibility = ["//visibility:public"],
)
