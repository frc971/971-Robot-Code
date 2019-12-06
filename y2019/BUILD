load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

robot_downloader(
    data = [
        ":config.json",
    ],
    dirs = [
        "//y2019/vision/server:www_files",
    ],
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2019/control_loops/drivetrain:drivetrain",
        "//y2019/control_loops/superstructure:superstructure",
        "//y2019/actors:binaries",
        "//y2019/vision/server",
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
        "//y2019/control_loops/drivetrain:camera",
        "//y2019/control_loops/drivetrain:polydrivetrain_plants",
        "//y2019/control_loops/superstructure/elevator:elevator_plants",
        "//y2019/control_loops/superstructure/intake:intake_plants",
        "//y2019/control_loops/superstructure/stilts:stilts_plants",
        "//y2019/control_loops/superstructure/wrist:wrist_plants",
        "//y2019/vision:constants",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":camera_log_fbs",
        ":constants",
        ":status_light_fbs",
        "//aos:init",
        "//aos:make_unique",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/robot_state:robot_state_fbs",
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
        "//y2019/control_loops/drivetrain:camera_fbs",
        "//y2019/control_loops/superstructure:superstructure_output_fbs",
        "//y2019/control_loops/superstructure:superstructure_position_fbs",
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
        ":camera_log_fbs",
        ":vision_proto",
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
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:localizer_fbs",
        "//y2019/control_loops/drivetrain:drivetrain_base",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
        "//y2019/control_loops/superstructure:superstructure_goal_fbs",
        "//y2019/control_loops/superstructure:superstructure_position_fbs",
        "//y2019/control_loops/superstructure:superstructure_status_fbs",
        "@com_google_protobuf//:protobuf",
    ],
)

flatbuffer_cc_library(
    name = "camera_log_fbs",
    srcs = [
        "camera_log.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "status_light_fbs",
    srcs = [
        "status_light.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

aos_config(
    name = "config",
    src = "y2019.json",
    flatbuffers = [
        ":status_light_fbs",
        "//y2019/control_loops/drivetrain:camera_fbs",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
        "//y2019/control_loops/superstructure:superstructure_goal_fbs",
        "//y2019/control_loops/superstructure:superstructure_output_fbs",
        "//y2019/control_loops/superstructure:superstructure_position_fbs",
        "//y2019/control_loops/superstructure:superstructure_status_fbs",
        ":camera_log_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/robot_state:config",
        "//frc971/autonomous:config",
        "//frc971/control_loops/drivetrain:config",
        "//frc971/wpilib:config",
    ],
)

cc_proto_library(
    name = "vision_proto",
    srcs = ["vision.proto"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
