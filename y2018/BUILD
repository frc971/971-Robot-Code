load("//aos/downloader:downloader.bzl", "aos_downloader")
load("//aos/build:queues.bzl", "queue_library")
load("//tools/build_rules:protobuf.bzl", "proto_cc_library")

aos_downloader(
    name = "download",
    srcs = [
        "//aos:prime_binaries",
    ],
    restricted_to = ["//tools:roborio"],
    start_srcs = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2018/vision:vision_status",
        "//aos:prime_start_binaries",
        "//y2018/control_loops/drivetrain:drivetrain",
        "//y2018/actors:autonomous_action",
        "//y2018/control_loops/superstructure:superstructure",
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
        "//y2018/vision:vision_status.stripped",
        "//aos:prime_start_binaries_stripped",
        "//y2018/actors:autonomous_action.stripped",
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
        ":vision_proto",
        "//aos/stl_mutex:stl_mutex",
        "//aos/time:time",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/network:team_number",
        "//aos/util:log_interval",
        "//aos/input:drivetrain_input",
        "//aos/input:joystick_input",
        "//aos/linux_code:init",
        "//aos/vision/events:udp",
        "//frc971/autonomous:auto_queue",
        "//frc971/autonomous:base_autonomous_actor",
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
        "//aos/mutex:mutex",
        "//aos/logging",
        "//aos/network:team_number",
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
        ":status_light",
        "//aos:math",
        "//aos/stl_mutex:stl_mutex",
        "//aos/time:time",
        "//aos/controls:control_loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state:robot_state",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//aos/linux_code:init",
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//third_party/Phoenix-frc-lib:phoenix",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_queue",
        "//y2018/vision:vision_queue",
    ],
)

queue_library(
    name = "status_light",
    srcs = [
        "status_light.q",
    ],
    visibility = ["//visibility:public"],
)

proto_cc_library(
    name = "vision_proto",
    src = "vision.proto",
    visibility = ["//visibility:public"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
