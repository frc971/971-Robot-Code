load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("//aos/util:config_validator_macro.bzl", "config_validator_test")

config_validator_test(
    name = "config_validator_test",
    config = "//y2023_bot3:aos_config",
)

robot_downloader(
    binaries = [
        "//aos/network:web_proxy_main",
        "//aos/events/logging:log_cat",
        "//y2023_bot3/constants:constants_sender",
        "//aos/events:aos_timing_report_streamer",
    ],
    data = [
        ":aos_config",
        "//aos/starter:roborio_irq_config.json",
        "@ctre_phoenix6_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix6_tools_athena//:shared_libraries",
        "@ctre_phoenix_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix_cci_athena//:shared_libraries",
    ],
    dirs = [
        "//y2023_bot3/www:www_files",
        "//y2023_bot3/autonomous:splines",
    ],
    start_binaries = [
        "//aos/events/logging:logger_main",
        "//aos/network:web_proxy_main",
        "//aos/starter:irq_affinity",
        "//y2023_bot3/autonomous:binaries",
        ":joystick_reader",
        ":wpilib_interface",
        "//frc971/can_logger",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//y2023_bot3/control_loops/drivetrain:drivetrain",
        "//y2023_bot3/control_loops/drivetrain:trajectory_generator",
        "//y2023_bot3/control_loops/superstructure:superstructure",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

robot_downloader(
    name = "pi_download",
    binaries = [
        "//aos/starter:irq_affinity",
        "//aos/util:foxglove_websocket",
        "//aos/events:aos_timing_report_streamer",
        "//y2023_bot3/constants:constants_sender",
        "//aos/network:web_proxy_main",
        "//aos/events/logging:log_cat",
        "//y2023_bot3/rockpi:imu_main",
        "//frc971/image_streamer:image_streamer",
    ],
    data = [
        ":aos_config",
        "//frc971/rockpi:rockpi_config.json",
        "//y2023_bot3/constants:constants.json",
        "//y2023_bot3/www:www_files",
    ],
    dirs = [
        "//y2023_bot3/www:www_files",
        "//frc971/image_streamer/www:www_files",
    ],
    start_binaries = [
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//aos/network:web_proxy_main",
        "//aos/starter:irq_affinity",
        "//aos/events/logging:logger_main",
    ],
    target_compatible_with = ["//tools/platforms/hardware:raspberry_pi"],
    target_type = "pi",
)

aos_config(
    name = "aos_config",
    src = "y2023_bot3.json",
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
        ":config_roborio",
    ],
)

aos_config(
    name = "config_imu",
    src = "y2023_bot3_imu.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//y2023_bot3/constants:constants_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
    ],
)

aos_config(
    name = "config_roborio",
    src = "y2023_bot3_roborio.json",
    flatbuffers = [
        "//aos/network:remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//y2023_bot3/constants:constants_fbs",
        "//aos/network:timestamp_fbs",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
        "//y2023_bot3/control_loops/superstructure:superstructure_goal_fbs",
        "//y2023_bot3/control_loops/superstructure:superstructure_output_fbs",
        "//y2023_bot3/control_loops/superstructure:superstructure_position_fbs",
        "//y2023_bot3/control_loops/superstructure:superstructure_status_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc971:can_configuration_fbs",
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

cc_library(
    name = "constants",
    srcs = [
        "constants.cc",
    ],
    hdrs = [
        "constants.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/mutex",
        "//aos/network:team_number",
        "//frc971:constants",
        "//frc971/control_loops:pose",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc971/shooter_interpolation:interpolation",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//y2023_bot3/control_loops/drivetrain:polydrivetrain_plants",
        "//y2023_bot3/control_loops/superstructure/pivot_joint:pivot_joint_plants",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":constants",
        "//aos:init",
        "//aos:math",
        "//aos/containers:sized_array",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971:can_configuration_fbs",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/input:robot_state_fbs",
        "//frc971/queues:gyro_fbs",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:phoenix",
        "//third_party:phoenix6",
        "//third_party:wpilib",
        "//y2023_bot3/control_loops/superstructure:led_indicator_lib",
        "//y2023_bot3/control_loops/superstructure:superstructure_output_fbs",
        "//y2023_bot3/control_loops/superstructure:superstructure_position_fbs",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        ":joystick_reader.cc",
    ],
    deps = [
        ":constants",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/input:action_joystick_input",
        "//frc971/input:drivetrain_input",
        "//frc971/input:joystick_input",
        "//frc971/input:redundant_joystick_data",
        "//y2023_bot3/control_loops/drivetrain:drivetrain_base",
        "//y2023_bot3/control_loops/superstructure:superstructure_goal_fbs",
        "//y2023_bot3/control_loops/superstructure:superstructure_status_fbs",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
