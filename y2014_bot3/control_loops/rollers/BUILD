package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "rollers_goal_fbs",
    srcs = [
        "rollers_goal.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "rollers_position_fbs",
    srcs = [
        "rollers_position.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "rollers_output_fbs",
    srcs = [
        "rollers_output.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "rollers_status_fbs",
    srcs = [
        "rollers_status.fbs",
    ],
    gen_reflections = 1,
)

cc_library(
    name = "rollers_lib",
    srcs = [
        "rollers.cc",
    ],
    hdrs = [
        "rollers.h",
    ],
    deps = [
        ":rollers_goal_fbs",
        ":rollers_output_fbs",
        ":rollers_position_fbs",
        ":rollers_status_fbs",
        "//aos/controls:control_loop",
        "//aos/logging",
    ],
)

cc_binary(
    name = "rollers",
    srcs = [
        "rollers_main.cc",
    ],
    deps = [
        ":rollers_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
