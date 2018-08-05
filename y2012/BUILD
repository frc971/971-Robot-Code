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
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/queues:gyro",
        "//y2012/control_loops/accessories:accessories_queue",
    ],
)

aos_downloader(
    name = "download",
    srcs = [
        "//aos:prime_binaries",
    ],
    restricted_to = ["//tools:roborio"],
    start_srcs = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2012/control_loops/drivetrain",
        "//y2012/control_loops/accessories",
        "//aos:prime_start_binaries",
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
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:gyro_sender",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2012/control_loops/accessories:accessories_queue",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
