load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "auto_mode_fbs",
    srcs = [
        "auto_mode.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "hot_goal_fbs",
    srcs = [
        "hot_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
