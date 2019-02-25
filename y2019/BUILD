load("//frc971:downloader.bzl", "robot_downloader")

robot_downloader(
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2019/control_loops/drivetrain:drivetrain",
        "//y2019/control_loops/superstructure:superstructure",
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
        "//aos/logging",
        "//aos/mutex",
        "//aos/network:team_number",
        "//frc971:constants",
        "//frc971/control_loops:pose",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//y2019/control_loops/drivetrain:camera",
        "//y2019/control_loops/drivetrain:polydrivetrain_plants",
        "//y2019/control_loops/superstructure/elevator:elevator_plants",
        "//y2019/control_loops/superstructure/intake:intake_plants",
        "//y2019/control_loops/superstructure/stilts:stilts_plants",
        "//y2019/control_loops/superstructure/wrist:wrist_plants",
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
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_queue",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//third_party/Phoenix-frc-lib:phoenix",
        "//y2019/control_loops/superstructure:superstructure_queue",
        "//y2019/jevois:spi",
    ],
)

cc_library(
    name = "joystick_angle",
    srcs = [
        "joystick_angle.cc",
    ],
    hdrs = [
        "joystick_angle.h",
    ],
    deps = [
        "//aos/input:drivetrain_input",
        "//frc971/zeroing:wrap",
    ],
)

cc_test(
    name = "joystick_angle_test",
    srcs = [
        "joystick_angle_test.cc",
    ],
    deps = [
        ":joystick_angle",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        ":joystick_reader.cc",
    ],
    deps = [
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/input:action_joystick_input",
        "//aos/input:drivetrain_input",
        "//aos/input:joystick_input",
        "//aos/logging",
        "//aos/network:team_number",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/vision/events:udp",
        "//frc971/autonomous:auto_queue",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2019/control_loops/drivetrain:drivetrain_base",
        "//y2019/control_loops/superstructure:superstructure_queue",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
