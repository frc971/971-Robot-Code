package(default_visibility = ["//visibility:public"])

load("//aos:config.bzl", "aos_config")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_library(
    name = "joystick_input",
    srcs = [
        "joystick_input.cc",
    ],
    hdrs = [
        "joystick_input.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":robot_state_fbs",
        "//aos/events:event_loop",
        "//frc971/input:driver_station_data",
        "//aos/logging",
    ],
)

cc_library(
    name = "drivetrain_input",
    srcs = [
        "drivetrain_input.cc",
    ],
    hdrs = [
        "drivetrain_input.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":robot_state_fbs",
        "//aos:math",
        "//frc971/input:driver_station_data",
        "//aos/logging",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_goal_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/control_loops/drivetrain:spline_goal_fbs",
    ],
)

cc_library(
    name = "driver_station_data",
    srcs = [
        "driver_station_data.cc",
    ],
    hdrs = [
        "driver_station_data.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":joystick_state_fbs",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "action_joystick_input",
    srcs = ["action_joystick_input.cc"],
    hdrs = ["action_joystick_input.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_input",
        "//aos:init",
        "//aos/actions:action_lib",
        "//frc971/input:joystick_input",
        "//aos/logging",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/autonomous:base_autonomous_actor",
    ],
)

flatbuffer_cc_library(
    name = "robot_state_fbs",
    srcs = ["robot_state.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "joystick_state_fbs",
    srcs = ["joystick_state.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "config",
    src = "robot_state_config.json",
    flatbuffers = [
        ":joystick_state_fbs",
        ":robot_state_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//aos/events:config"],
)