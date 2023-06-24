load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("//aos/util:config_validator_macro.bzl", "config_validator_test")

config_validator_test(
    name = "config_validator_test",
    config = "//y2023_bot4:aos_config",
)

robot_downloader(
    binaries = [
        "//aos/network:web_proxy_main",
        "//aos/events/logging:log_cat",
        "//aos/events:aos_timing_report_streamer",
    ],
    data = [
        ":aos_config",
        ":swerve_publisher_output_json",
        "@ctre_phoenix6_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix6_tools_athena//:shared_libraries",
        "@ctre_phoenix_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix_cci_athena//:shared_libraries",
    ],
    start_binaries = [
        "//aos/events/logging:logger_main",
        "//aos/network:web_proxy_main",
        "//aos/starter:irq_affinity",
        ":wpilib_interface",
        ":swerve_publisher",
        "//frc971/can_logger",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

robot_downloader(
    name = "pi_download",
    binaries = [
        "//aos/util:foxglove_websocket",
        "//aos/events:aos_timing_report_streamer",
        "//aos/events/logging:log_cat",
        "//y2023/rockpi:imu_main",
        "//frc971/image_streamer:image_streamer",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//aos/network:web_proxy_main",
        "//aos/starter:irq_affinity",
        "//aos/events/logging:logger_main",
    ],
    data = [
        ":aos_config",
        "//frc971/rockpi:rockpi_config.json",
    ],
    start_binaries = [
    ],
    target_compatible_with = ["//tools/platforms/hardware:raspberry_pi"],
    target_type = "pi",
)

filegroup(
    name = "swerve_publisher_output_json",
    srcs = [
        "swerve_drivetrain_output.json",
    ],
    visibility = ["//y2023_bot4:__subpackages__"],
)

cc_library(
    name = "constants",
    srcs = ["constants.cc"],
    hdrs = [
        "constants.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/network:team_number",
        "//frc971:constants",
    ],
)

flatbuffer_cc_library(
    name = "drivetrain_position_fbs",
    srcs = ["drivetrain_position.fbs"],
    gen_reflections = 1,
    deps = ["//frc971/control_loops:control_loops_fbs"],
)

flatbuffer_cc_library(
    name = "drivetrain_can_position_fbs",
    srcs = ["drivetrain_can_position.fbs"],
    gen_reflections = 1,
    deps = ["//frc971/control_loops:can_falcon_fbs"],
)

cc_binary(
    name = "swerve_publisher",
    srcs = ["swerve_publisher_main.cc"],
    deps = [
        ":swerve_publisher_lib",
        "//aos/events:shm_event_loop",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "swerve_publisher_lib",
    srcs = ["swerve_publisher_lib.cc"],
    hdrs = ["swerve_publisher_lib.h"],
    deps = [
        "//aos:init",
        "//aos/events:event_loop",
        "//frc971/control_loops/drivetrain/swerve:swerve_drivetrain_output_fbs",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "swerve_publisher_lib_test",
    srcs = [
        "swerve_publisher_lib_test.cc",
    ],
    data = [
        ":aos_config",
        ":swerve_publisher_output_json",
    ],
    deps = [
        ":swerve_publisher_lib",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = ["wpilib_interface.cc"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":constants",
        ":drivetrain_can_position_fbs",
        ":drivetrain_position_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/wpilib:can_sensor_reader",
        "//frc971/wpilib:falcon",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_robot_base",
        "//frc971/wpilib/swerve:swerve_drivetrain_writer",
    ],
)

aos_config(
    name = "aos_config",
    src = "y2023_bot4.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc971/input:robot_state_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":config_imu",
        ":config_logger",
        ":config_roborio",
    ],
)

aos_config(
    name = "config_roborio",
    src = "y2023_bot4_roborio.json",
    flatbuffers = [
        ":drivetrain_position_fbs",
        ":drivetrain_can_position_fbs",
        "//frc971:can_configuration_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc971/control_loops/drivetrain/swerve:swerve_drivetrain_output_fbs",
        "//frc971/control_loops/drivetrain/swerve:swerve_drivetrain_position_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc971/can_logger:can_logging_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/autonomous:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
        "//frc971/input:aos_config",
        "//frc971/wpilib:aos_config",
    ],
)

aos_config(
    name = "config_imu",
    src = "y2023_bot4_imu.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc971/vision:target_map_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
    ],
)

aos_config(
    name = "config_logger",
    src = "y2023_bot4_logger.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:target_map_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
        "//frc971/input:aos_config",
    ],
)
