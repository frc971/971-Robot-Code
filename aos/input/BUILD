package(default_visibility = ["//visibility:public"])

cc_library(
    name = "joystick_input",
    srcs = [
        "joystick_input.cc",
    ],
    hdrs = [
        "joystick_input.h",
    ],
    deps = [
        "//aos/events:event_loop",
        "//aos/input:driver_station_data",
        "//aos/logging",
        "//aos/robot_state:robot_state_fbs",
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
    deps = [
        "//aos:math",
        "//aos/input:driver_station_data",
        "//aos/logging",
        "//aos/robot_state:robot_state_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_goal_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
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
    deps = [
        "//aos/robot_state:joystick_state_fbs",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "action_joystick_input",
    srcs = ["action_joystick_input.cc"],
    hdrs = ["action_joystick_input.h"],
    deps = [
        ":drivetrain_input",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/input:joystick_input",
        "//aos/logging",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/autonomous:base_autonomous_actor",
    ],
)
