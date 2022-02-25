load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

robot_downloader(
    start_binaries = [
        ":joystick_reader",
        ":wpilib_interface",
        "//y2018/vision:vision_status",
        "//y2018/actors:autonomous_action",
        "//y2018/control_loops/drivetrain:drivetrain",
        "//y2018/control_loops/superstructure:superstructure",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        "joystick_reader.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":vision_proto",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/network:team_number",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/vision/events:udp",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/input:action_joystick_input",
        "//frc971/input:drivetrain_input",
        "//y2018/control_loops/drivetrain:drivetrain_base",
        "//y2018/control_loops/superstructure:superstructure_goal_fbs",
        "//y2018/control_loops/superstructure:superstructure_position_fbs",
        "//y2018/control_loops/superstructure:superstructure_status_fbs",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/network:team_number",
        "//aos/stl_mutex",
        "//frc971:constants",
        "//frc971/shooter_interpolation:interpolation",
        "//y2018/control_loops/drivetrain:polydrivetrain_plants",
        "//y2018/control_loops/superstructure/arm:dynamics",
        "//y2018/control_loops/superstructure/intake:intake_plants",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":status_light_fbs",
        "//aos:init",
        "//aos:math",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/input:robot_state_fbs",
        "//frc971/wpilib:ADIS16448",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:dma",
        "//frc971/wpilib:dma_edge_counting",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:phoenix",
        "//third_party:wpilib",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_output_fbs",
        "//y2018/control_loops/superstructure:superstructure_position_fbs",
        "//y2018/vision:vision_fbs",
    ],
)

flatbuffer_cc_library(
    name = "status_light_fbs",
    srcs = [
        "status_light.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

aos_config(
    name = "aos_config",
    src = "y2018.json",
    flatbuffers = [
        ":status_light_fbs",
        "//y2018/control_loops/superstructure:superstructure_goal_fbs",
        "//y2018/control_loops/superstructure:superstructure_output_fbs",
        "//y2018/control_loops/superstructure:superstructure_position_fbs",
        "//y2018/control_loops/superstructure:superstructure_status_fbs",
        "//y2018/vision:vision_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:aos_config",
        "//frc971/input:aos_config",
    ],
)

cc_proto_library(
    name = "vision_proto",
    srcs = ["vision.proto"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
