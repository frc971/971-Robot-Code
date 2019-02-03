load("//aos/downloader:downloader.bzl", "aos_downloader")

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    deps = [
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/input:drivetrain_input",
        "//aos/input:joystick_input",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc971/autonomous:auto_queue",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2017_bot3/control_loops/drivetrain:drivetrain_base",
        "//y2017_bot3/control_loops/superstructure:superstructure_lib",
        "//y2017_bot3/control_loops/superstructure:superstructure_queue",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos:init",
        "//aos:make_unique",
        "//aos/controls:control_loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:gyro_sender",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2017_bot3/control_loops/drivetrain:polydrivetrain_plants",
        "//y2017_bot3/control_loops/superstructure:superstructure_queue",
    ],
)

aos_downloader(
    name = "download",
    srcs = [
        "//aos:prime_binaries",
    ],
    restricted_to = ["//tools:roborio"],
    start_srcs = [
        ":wpilib_interface",
        ":joystick_reader",
        "//aos:prime_start_binaries",
        "//y2017_bot3/control_loops/drivetrain:drivetrain",
        "//y2017_bot3/control_loops/superstructure:superstructure",
    ],
)

aos_downloader(
    name = "download_stripped",
    srcs = [
        "//aos:prime_binaries_stripped",
    ],
    restricted_to = ["//tools:roborio"],
    start_srcs = [
        ":wpilib_interface.stripped",
        ":joystick_reader.stripped",
        "//aos:prime_start_binaries_stripped",
        "//y2017_bot3/control_loops/drivetrain:drivetrain.stripped",
        "//y2017_bot3/control_loops/superstructure:superstructure.stripped",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
