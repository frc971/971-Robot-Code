load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")

cc_library(
    name = "constants",
    srcs = [
        "constants.cc",
    ],
    hdrs = [
        "constants.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/network:team_number",
        "//aos/stl_mutex",
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014/control_loops/drivetrain:polydrivetrain_plants",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":constants",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc971/autonomous:auto_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/input:action_joystick_input",
        "//frc971/queues:gyro_fbs",
        "//y2014/actors:shoot_action_lib",
        "//y2014/control_loops/claw:claw_goal_fbs",
        "//y2014/control_loops/claw:claw_status_fbs",
        "//y2014/control_loops/drivetrain:drivetrain_base",
        "//y2014/control_loops/shooter:shooter_goal_fbs",
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
        "//y2014/actors:binaries",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "hot_goal_reader",
    srcs = [
        "hot_goal_reader.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/time",
        "//y2014/queues:hot_goal_fbs",
    ],
)

aos_config(
    name = "aos_config",
    src = "y2014.json",
    flatbuffers = [
        "//y2014/control_loops/shooter:shooter_goal_fbs",
        "//y2014/control_loops/shooter:shooter_output_fbs",
        "//y2014/control_loops/shooter:shooter_position_fbs",
        "//y2014/control_loops/shooter:shooter_status_fbs",
        "//y2014/control_loops/claw:claw_goal_fbs",
        "//y2014/control_loops/claw:claw_output_fbs",
        "//y2014/control_loops/claw:claw_position_fbs",
        "//y2014/control_loops/claw:claw_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:aos_config",
        "//frc971/input:aos_config",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":constants",
        "//aos:init",
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
        "//frc971/wpilib:gyro_sender",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:wpilib",
        "//y2014/control_loops/claw:claw_output_fbs",
        "//y2014/control_loops/claw:claw_position_fbs",
        "//y2014/control_loops/shooter:shooter_output_fbs",
        "//y2014/control_loops/shooter:shooter_position_fbs",
        "//y2014/queues:auto_mode_fbs",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
