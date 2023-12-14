load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "rollers_goal_fbs",
    srcs = [
        "rollers_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "rollers_position_fbs",
    srcs = [
        "rollers_position.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "rollers_output_fbs",
    srcs = [
        "rollers_output.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "rollers_status_fbs",
    srcs = [
        "rollers_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "rollers_lib",
    srcs = [
        "rollers.cc",
    ],
    hdrs = [
        "rollers.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":rollers_goal_fbs",
        ":rollers_output_fbs",
        ":rollers_position_fbs",
        ":rollers_status_fbs",
        "//aos/logging",
        "//frc971/control_loops:control_loop",
    ],
)

cc_binary(
    name = "rollers",
    srcs = [
        "rollers_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":rollers_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
