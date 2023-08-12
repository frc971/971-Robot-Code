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
        "//frc971/control_loops/drivetrain/swerve:swerve_drivetrain_output_fbs",
        "//frc971/wpilib:falcon",
        "//frc971/wpilib:loop_output_handler",
        "//third_party:phoenix6",
    ],
)

cc_library(
    name = "swerve_module",
    hdrs = [
        "swerve_module.h",
    ],
    deps = [
        "//frc971/wpilib:falcon",
    ],
)
