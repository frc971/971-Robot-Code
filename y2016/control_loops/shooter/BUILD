package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "shooter_goal_fbs",
    srcs = [
        "shooter_goal.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "shooter_position_fbs",
    srcs = [
        "shooter_position.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "shooter_output_fbs",
    srcs = [
        "shooter_output.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "shooter_status_fbs",
    srcs = [
        "shooter_status.fbs",
    ],
    gen_reflections = 1,
)

genrule(
    name = "genrule_shooter",
    outs = [
        "shooter_plant.h",
        "shooter_plant.cc",
        "shooter_integral_plant.h",
        "shooter_integral_plant.cc",
    ],
    cmd = "$(location //y2016/control_loops/python:shooter) $(OUTS)",
    tools = [
        "//y2016/control_loops/python:shooter",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "shooter_plants",
    srcs = [
        "shooter_integral_plant.cc",
        "shooter_plant.cc",
    ],
    hdrs = [
        "shooter_integral_plant.h",
        "shooter_plant.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "shooter_lib",
    srcs = [
        "shooter.cc",
    ],
    hdrs = [
        "shooter.h",
    ],
    deps = [
        ":shooter_goal_fbs",
        ":shooter_output_fbs",
        ":shooter_plants",
        ":shooter_position_fbs",
        ":shooter_status_fbs",
        "//aos/controls:control_loop",
    ],
)

cc_test(
    name = "shooter_lib_test",
    srcs = [
        "shooter_lib_test.cc",
    ],
    data = ["//y2016:config.json"],
    deps = [
        ":shooter_goal_fbs",
        ":shooter_lib",
        ":shooter_output_fbs",
        ":shooter_position_fbs",
        ":shooter_status_fbs",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "shooter",
    srcs = [
        "shooter_main.cc",
    ],
    deps = [
        ":shooter_goal_fbs",
        ":shooter_lib",
        ":shooter_output_fbs",
        ":shooter_position_fbs",
        ":shooter_status_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
