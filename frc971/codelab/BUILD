load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "basic_test",
    testonly = 1,
    srcs = ["basic_test.cc"],
    data = [":aos_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":basic",
        ":basic_goal_fbs",
        ":basic_output_fbs",
        ":basic_position_fbs",
        ":basic_status_fbs",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loop_test",
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
        "//frc971/control_loops:control_loop",
    ],
)

static_flatbuffer(
    name = "basic_goal_fbs",
    srcs = [
        "basic_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "basic_output_fbs",
    srcs = [
        "basic_output.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "basic_status_fbs",
    srcs = [
        "basic_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "basic_position_fbs",
    srcs = [
        "basic_position.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "aos_config",
    src = "codelab.json",
    flatbuffers = [
        ":basic_goal_fbs",
        ":basic_position_fbs",
        ":basic_status_fbs",
        ":basic_output_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/input:aos_config",
    ],
)
