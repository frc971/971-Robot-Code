package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

cc_binary(
    name = "basic_test",
    testonly = 1,
    srcs = ["basic_test.cc"],
    data = [":config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":basic",
        ":basic_goal_fbs",
        ":basic_output_fbs",
        ":basic_position_fbs",
        ":basic_status_fbs",
        "//aos/controls:control_loop_test",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_library(
    name = "basic",
    srcs = ["basic.cc"],
    hdrs = ["basic.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":basic_goal_fbs",
        ":basic_output_fbs",
        ":basic_position_fbs",
        ":basic_status_fbs",
        "//aos/controls:control_loop",
    ],
)

flatbuffer_cc_library(
    name = "basic_goal_fbs",
    srcs = [
        "basic_goal.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "basic_output_fbs",
    srcs = [
        "basic_output.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "basic_status_fbs",
    srcs = [
        "basic_status.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "basic_position_fbs",
    srcs = [
        "basic_position.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "config",
    src = "codelab.json",
    flatbuffers = [
        ":basic_goal_fbs",
        ":basic_position_fbs",
        ":basic_status_fbs",
        ":basic_output_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/robot_state:config",
    ],
)
