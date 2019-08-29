package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "auto_mode_fbs",
    srcs = [
        "auto_mode.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "hot_goal_fbs",
    srcs = [
        "hot_goal.fbs",
    ],
    gen_reflections = 1,
)
