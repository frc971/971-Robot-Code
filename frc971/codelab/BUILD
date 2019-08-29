package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_binary(
    name = "basic_test",
    testonly = 1,
    srcs = ["basic_test.cc"],
    deps = [
        ":basic",
        ":basic_fbs",
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
    deps = [
        ":basic_fbs",
        "//aos/controls:control_loop",
    ],
)

flatbuffer_cc_library(
    name = "basic_fbs",
    srcs = [
        "basic.fbs",
    ],
)
