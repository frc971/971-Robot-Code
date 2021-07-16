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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/logging",
        "//aos/network:team_number",
        "//aos/stl_mutex",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":constants",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc971/input:action_joystick_input",
        "//frc971/input:drivetrain_input",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:config",
        "//frc971/input:config",
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
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/input:robot_state_fbs",
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
    target_compatible_with = ["@platforms//os:linux"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
