load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "auto_mode_fbs",
    srcs = [
        "auto_mode.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "hot_goal_fbs",
    srcs = [
        "hot_goal.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)
