load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

robot_downloader(
    data = [
        ":config.json",
    ],
    start_binaries = [
        "//aos/events/logging:logger_main",
        ":joystick_reader",
        ":wpilib_interface",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//y2020/actors:binaries",
        "//y2020/control_loops/drivetrain:drivetrain",
        "//y2020/control_loops/superstructure:superstructure",
    ],
)

robot_downloader(
    name = "pi_download",
    data = [
        ":config.json",
        "//y2020/www:files",
        "//y2020/www:flatbuffers",
        "//y2020/www:main_bundle",
    ],
    dirs = [
        "//y2020/www:www_files",
    ],
    restricted_to = ["//tools:armhf-debian"],
    start_binaries = [
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//y2020/vision:camera_reader",
        "//aos/network:web_proxy_main",
    ],
    target_type = "pi",
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
        "//aos/logging",
        "//aos/mutex",
        "//aos/network:team_number",
        "//frc971:constants",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//y2020/control_loops/drivetrain:polydrivetrain_plants",
        "//y2020/control_loops/superstructure/accelerator:accelerator_plants",
        "//y2020/control_loops/superstructure/control_panel:control_panel_plants",
        "//y2020/control_loops/superstructure/finisher:finisher_plants",
        "//y2020/control_loops/superstructure/hood:hood_plants",
        "//y2020/control_loops/superstructure/intake:intake_plants",
        "//y2020/control_loops/superstructure/turret:turret_plants",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":constants",
        "//aos:init",
        "//aos:make_unique",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/robot_state:robot_state_fbs",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/wpilib:ADIS16470",
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
        "//third_party:wpilib",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        ":joystick_reader.cc",
    ],
    deps = [
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/input:action_joystick_input",
        "//aos/input:drivetrain_input",
        "//aos/input:joystick_input",
        "//aos/logging",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//y2020:constants",
        "//y2020/control_loops/drivetrain:drivetrain_base",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
    ],
)

aos_config(
    name = "config",
    src = "y2020.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision:vision_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/robot_state:config",
        "//frc971/autonomous:config",
        "//frc971/control_loops/drivetrain:config",
        "//frc971/wpilib:config",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "web_proxy",
    srcs = ["web_proxy.sh"],
    data = [
        ":config.json",
        "//aos/network:web_proxy_main",
        "//y2020/www:files",
        "//y2020/www:flatbuffers",
        "//y2020/www:main_bundle",
    ],
)
