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
        "//y2019/control_loops/drivetrain:polydrivetrain_plants",
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
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
