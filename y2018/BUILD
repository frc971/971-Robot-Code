load("//aos/downloader:downloader.bzl", "aos_downloader")

aos_downloader(
    name = "download",
    srcs = [
        "//aos:prime_binaries",
    ],
    restricted_to = ["//tools:roborio"],
    start_srcs = [
        ":joystick_reader",
        ":wpilib_interface",
        "//aos:prime_start_binaries",
        "//y2018/control_loops/drivetrain:drivetrain",
        "//y2018/control_loops/superstructure:superstructure.stripped",
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
        "//y2018/control_loops/drivetrain:drivetrain.stripped",
        "//y2018/control_loops/superstructure:superstructure.stripped",
    ],
)

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
        "//aos/input:drivetrain_input",
        "//aos/input:joystick_input",
        "//aos/linux_code:init",
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2018/control_loops/drivetrain:drivetrain_base",
        "//y2018/control_loops/superstructure:superstructure_queue",
        "//y2018/control_loops/superstructure/arm:generated_graph",
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
        "//aos:once",
        "//aos/common:mutex",
        "//aos/common/logging",
        "//aos/common/network:team_number",
        "//frc971:constants",
        "//frc971/shooter_interpolation:interpolation",
        "//y2018/control_loops/drivetrain:polydrivetrain_plants",
        "//y2018/control_loops/superstructure/arm:dynamics",
        "//y2018/control_loops/superstructure/intake:intake_plants",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/common:math",
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
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:gyro_sender",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_queue",
    ],
)
