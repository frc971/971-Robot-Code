load('//frc971:downloader.bzl', 'robot_downloader')

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
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:state_feedback_loop",
        "//y2016/control_loops/drivetrain:polydrivetrain_plants",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    deps = [
        ":constants",
        "//aos/time:time",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/util:log_interval",
        "//aos/input:joystick_input",
        "//aos:init",
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/queues:gyro",
        "//y2016/actors:autonomous_action_lib",
        "//y2016/actors:superstructure_action_lib",
        "//y2016/actors:vision_align_action_lib",
        "//y2016/control_loops/shooter:shooter_queue",
        "//y2016/control_loops/superstructure:superstructure_lib",
        "//y2016/control_loops/superstructure:superstructure_queue",
        "//y2016/queues:ball_detector",
    ],
)

robot_downloader(
  start_binaries = [
    ':joystick_reader',
    ':wpilib_interface',
    '//y2016/control_loops/drivetrain:drivetrain',
    '//y2016/control_loops/superstructure:superstructure',
    '//y2016/control_loops/shooter:shooter',
    '//y2016/dashboard:dashboard',
    '//y2016/actors:autonomous_action',
    '//y2016/actors:superstructure_action',
    '//y2016/actors:vision_align_action',
    '//y2016/vision:target_receiver',
  ],
  dirs = [
    '//y2016/dashboard:www_files',
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
        "//aos:init",
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
        "//y2016/control_loops/drivetrain:polydrivetrain_plants",
        "//y2016/control_loops/shooter:shooter_queue",
        "//y2016/control_loops/superstructure:superstructure_queue",
        "//y2016/queues:ball_detector",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
