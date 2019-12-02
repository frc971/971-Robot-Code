package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

cc_library(
    name = "action_lib",
    srcs = [
        "actions.cc",
        "actor.cc",
    ],
    hdrs = [
        "actions.h",
        "actor.h",
    ],
    deps = [
        ":actions_fbs",
        "//aos/controls:control_loop",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
    ],
)

flatbuffer_cc_library(
    name = "actions_fbs",
    srcs = ["actions.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "test_action_fbs",
    srcs = ["test_action.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "test_action2_fbs",
    srcs = ["test_action2.fbs"],
    gen_reflections = 1,
)

aos_config(
    name = "action_test_config",
    src = "action_test_config_source.json",
    flatbuffers = [
        ":actions_fbs",
        ":test_action_fbs",
        ":test_action2_fbs",
    ],
    deps = [
        "//aos/events:config",
    ],
)

cc_test(
    name = "action_test",
    srcs = [
        "action_test.cc",
    ],
    data = ["action_test_config.json"],
    deps = [
        ":action_lib",
        ":actions_fbs",
        ":test_action2_fbs",
        ":test_action_fbs",
        "//aos:event",
        "//aos/events:simulated_event_loop",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)
