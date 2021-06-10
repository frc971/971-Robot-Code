load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

robot_downloader(
    data = [
        ":config",
    ],
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2021_bot3/control_loops/drivetrain:drivetrain",
        "//y2021_bot3/control_loops/superstructure:superstructure",
        "//y2021_bot3/actors:binaries",
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
        "//aos/logging",
        "//aos/mutex",
        "//aos/network:team_number",
        "//frc971:constants",
        "//frc971/control_loops:pose",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//y2021_bot3/control_loops/drivetrain:polydrivetrain_plants",
        "@com_google_absl//absl/base",
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
        "//aos:make_unique",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//frc971/input:robot_state_fbs",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:phoenix",
        "//third_party:wpilib",
        "//y2021_bot3/control_loops/superstructure:superstructure_output_fbs",
        "//y2021_bot3/control_loops/superstructure:superstructure_position_fbs",
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
        "//frc971/input:action_joystick_input",
        "//frc971/input:drivetrain_input",
        "//frc971/input:joystick_input",
        "//aos/logging",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//y2021_bot3/control_loops/drivetrain:drivetrain_base",
        "//y2021_bot3/control_loops/superstructure:superstructure_goal_fbs",
        "//y2021_bot3/control_loops/superstructure:superstructure_status_fbs",
    ],
)

aos_config(
    name = "config",
    src = "y2021_bot3.json",
    flatbuffers = [
        "//y2021_bot3/control_loops/superstructure:superstructure_goal_fbs",
        "//y2021_bot3/control_loops/superstructure:superstructure_output_fbs",
        "//y2021_bot3/control_loops/superstructure:superstructure_position_fbs",
        "//y2021_bot3/control_loops/superstructure:superstructure_status_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/input:config",
        "//frc971/autonomous:config",
        "//frc971/control_loops/drivetrain:config",
        "//frc971/wpilib:config",
    ],
)
