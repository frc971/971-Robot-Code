load("//frc971:downloader.bzl", "robot_downloader")

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
        "//aos:once",
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
        "//aos/input:drivetrain_input",
        "//aos/input:joystick_input",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2017/actors:autonomous_action_lib",
        "//y2017/control_loops/drivetrain:drivetrain_base",
        "//y2017/control_loops/superstructure:superstructure_queue",
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
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2017/control_loops/superstructure:superstructure_queue",
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
