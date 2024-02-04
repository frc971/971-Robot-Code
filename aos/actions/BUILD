load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//aos:config.bzl", "aos_config")

package(default_visibility = ["//visibility:public"])

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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":actions_fbs",
        "//aos/events:event_loop",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
    ],
)

static_flatbuffer(
    name = "actions_fbs",
    srcs = ["actions.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "test_action_fbs",
    srcs = ["test_action.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "test_action2_fbs",
    srcs = ["test_action2.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "action_test_config",
    src = "action_test_config_source.json",
    flatbuffers = [
        ":actions_fbs",
        ":test_action_fbs",
        ":test_action2_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:aos_config",
    ],
)

cc_test(
    name = "action_test",
    srcs = [
        "action_test.cc",
    ],
    data = [":action_test_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":action_lib",
        ":actions_fbs",
        ":test_action2_fbs",
        ":test_action_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/time",
    ],
)
