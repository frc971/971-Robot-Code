package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "claw_goal_fbs",
    srcs = [
        "claw_goal.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "claw_position_fbs",
    srcs = [
        "claw_position.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "claw_output_fbs",
    srcs = [
        "claw_output.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "claw_status_fbs",
    srcs = [
        "claw_status.fbs",
    ],
    gen_reflections = 1,
)

genrule(
    name = "genrule_claw",
    outs = [
        "claw_motor_plant.h",
        "claw_motor_plant.cc",
    ],
    cmd = "$(location //y2014/control_loops/python:claw) $(OUTS)",
    tools = [
        "//y2014/control_loops/python:claw",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "claw_lib",
    srcs = [
        "claw.cc",
        "claw_motor_plant.cc",
    ],
    hdrs = [
        "claw.h",
        "claw_motor_plant.h",
    ],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":claw_goal_fbs",
        ":claw_output_fbs",
        ":claw_position_fbs",
        ":claw_status_fbs",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/controls:polytope",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:hall_effect_tracker",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014:constants",
    ],
)

cc_test(
    name = "claw_lib_test",
    srcs = [
        "claw_lib_test.cc",
    ],
    data = ["//y2014:config.json"],
    deps = [
        ":claw_goal_fbs",
        ":claw_lib",
        ":claw_output_fbs",
        ":claw_position_fbs",
        ":claw_status_fbs",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "claw",
    srcs = [
        "claw_main.cc",
    ],
    deps = [
        ":claw_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
