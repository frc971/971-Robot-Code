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
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014/control_loops/drivetrain:polydrivetrain_plants",
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
        "//aos/input:joystick_input",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc971/autonomous:auto_queue",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/queues:gyro",
        "//y2014/actors:shoot_action_lib",
        "//y2014/control_loops/claw:claw_queue",
        "//y2014/control_loops/shooter:shooter_queue",
    ],
)

robot_downloader(
    start_binaries = [
        ":hot_goal_reader",
        ":joystick_reader",
        ":wpilib_interface",
        "//y2014/control_loops/drivetrain:drivetrain",
        "//y2014/control_loops/claw:claw",
        "//y2014/control_loops/shooter:shooter",
        "//y2014/autonomous:auto",
        "//y2014/actors:binaries",
    ],
)

cc_binary(
    name = "hot_goal_reader",
    srcs = [
        "hot_goal_reader.cc",
    ],
    deps = [
        "//aos:init",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/time",
        "//y2014/queues:hot_goal",
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
        "//aos/controls:control_loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:gyro_sender",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2014/control_loops/claw:claw_queue",
        "//y2014/control_loops/shooter:shooter_queue",
        "//y2014/queues:auto_mode",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
