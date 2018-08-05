load("//aos/downloader:downloader.bzl", "aos_downloader")

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    deps = [
        "//aos/common:time",
        "//aos/common/actions:action_lib",
        "//aos/common/logging",
        "//aos/common/util:log_interval",
        "//aos/input:joystick_input",
        "//aos/linux_code:init",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/queues:gyro",
        "//y2014_bot3/autonomous:auto_queue",
        "//y2014_bot3/control_loops/rollers:rollers_queue",
    ],
)

aos_downloader(
    name = "download_stripped",
    srcs = [
        "//aos:prime_binaries_stripped",
    ],
    restricted_to = ["//tools:roborio"],
    start_srcs = [
        ":joystick_reader.stripped",
        ":wpilib_interface.stripped",
        "//aos:prime_start_binaries_stripped",
        "//y2014_bot3/autonomous:auto.stripped",
        "//y2014_bot3/control_loops/drivetrain:drivetrain.stripped",
        "//y2014_bot3/control_loops/rollers:rollers.stripped",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/common:stl_mutex",
        "//aos/common:time",
        "//aos/common/controls:control_loop",
        "//aos/common/logging",
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
        "//aos/common/util:log_interval",
        "//aos/common/util:phased_loop",
        "//aos/common/util:wrapping_counter",
        "//aos/linux_code:init",
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
