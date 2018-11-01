load("//frc971:downloader.bzl", "robot_downloader")

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    deps = [
        "//aos/time:time",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/util:log_interval",
        "//aos/input:joystick_input",
        "//aos:init",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/queues:gyro",
        "//y2014_bot3/autonomous:auto_queue",
        "//y2014_bot3/control_loops/rollers:rollers_queue",
    ],
)

robot_downloader(
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2014_bot3/autonomous:auto",
        "//y2014_bot3/control_loops/drivetrain:drivetrain",
        "//y2014_bot3/control_loops/rollers:rollers",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/stl_mutex:stl_mutex",
        "//aos/time:time",
        "//aos/controls:control_loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state:robot_state",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//aos:init",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:gyro_sender",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2014_bot3/autonomous:auto_queue",
        "//y2014_bot3/control_loops/drivetrain:drivetrain_base",
        "//y2014_bot3/control_loops/rollers:rollers_lib",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
