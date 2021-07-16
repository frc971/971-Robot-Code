load("//frc971:downloader.bzl", "robot_downloader")

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc971/autonomous:auto_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_goal_fbs",
        "//frc971/control_loops/drivetrain:spline_goal_fbs",
        "//frc971/input:joystick_input",
        "//y2012/control_loops/accessories:accessories_fbs",
    ],
)

robot_downloader(
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2012/control_loops/drivetrain",
        "//y2012/control_loops/accessories",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/input:robot_state_fbs",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2012/control_loops:control_loop_fbs",
        "//y2012/control_loops/accessories:accessories_fbs",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
