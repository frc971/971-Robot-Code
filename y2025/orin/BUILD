cc_binary(
    name = "can_logger",
    srcs = [
        "can_logger_main.cc",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "//frc971/can_logger:can_logger_lib",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "ctre_interface",
    srcs = ["ctre_interface.cc"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//frc971:can_configuration_fbs",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc971/wpilib:can_sensor_reader",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:talonfx",
        "//frc971/wpilib/swerve:swerve_drivetrain_writer",
        "//frc971/wpilib/swerve:swerve_module",
        "//y2025:constants",
        "//y2025/constants:constants_fbs",
    ],
)
