load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")

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
        "//frc971/shooter_interpolation:interpolation",
        "//y2017/control_loops/drivetrain:polydrivetrain_plants",
        "//y2017/control_loops/superstructure/column:column_plants",
        "//y2017/control_loops/superstructure/hood:hood_plants",
        "//y2017/control_loops/superstructure/intake:intake_plants",
        "//y2017/control_loops/superstructure/shooter:shooter_plants",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    deps = [
        ":constants",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/input:action_joystick_input",
        "//aos/input:drivetrain_input",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//y2017/actors:autonomous_action_lib",
        "//y2017/control_loops/drivetrain:drivetrain_base",
        "//y2017/control_loops/superstructure:superstructure_goal_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
    ],
)

aos_config(
    name = "config",
    src = "y2017.json",
    flatbuffers = [
        "//y2017/control_loops/superstructure:superstructure_goal_fbs",
        "//y2017/control_loops/superstructure:superstructure_output_fbs",
        "//y2017/control_loops/superstructure:superstructure_position_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
        "//y2017/vision:vision_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/robot_state:config",
        "//frc971/control_loops/drivetrain:config",
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
        "//aos/logging",
        "//aos/robot_state:robot_state_fbs",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2017/control_loops/superstructure:superstructure_output_fbs",
        "//y2017/control_loops/superstructure:superstructure_position_fbs",
    ],
)

robot_downloader(
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2017/control_loops/drivetrain:drivetrain",
        "//y2017/control_loops/superstructure:superstructure",
        "//y2017/actors:autonomous_action",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
