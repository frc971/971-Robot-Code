package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

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
        "//aos:queues",
        "//aos/controls:control_loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/time",
        "//aos/util:phased_loop",
    ],
)

queue_library(
    name = "action_queue",
    srcs = [
        "actions.q",
    ],
)

queue_library(
    name = "test_action_queue",
    srcs = [
        "test_action.q",
    ],
    deps = [
        ":action_queue",
    ],
)

cc_test(
    name = "action_test",
    srcs = [
        "action_test.cc",
    ],
    deps = [
        ":action_lib",
        ":action_queue",
        ":test_action_queue",
        "//aos:event",
        "//aos:queues",
        "//aos/events:simulated_event_loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)
