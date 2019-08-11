package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")
load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "replay_control_loop",
    hdrs = [
        "replay_control_loop.h",
    ],
    deps = [
        ":control_loop",
        "//aos:queues",
        "//aos/logging:queue_logging",
        "//aos/logging:replay",
        "//aos/time",
    ],
)

cc_library(
    name = "control_loop_test",
    testonly = True,
    srcs = [
        "control_loop_test.cc",
    ],
    hdrs = [
        "control_loop_test.h",
    ],
    deps = [
        "//aos/events:simulated_event_loop",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)

cc_library(
    name = "polytope_uc",
    hdrs = [
        "polytope.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//third_party/eigen",
    ],
)

cc_library(
    name = "polytope",
    hdrs = [
        "polytope.h",
    ],
    deps = [
        "//aos/logging",
        "//aos/logging:matrix_logging",
        "//third_party/cddlib",
        "//third_party/eigen",
    ],
)

cc_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.cc",
    ],
    deps = [
        ":polytope",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//third_party/eigen",
    ],
)

queue_library(
    name = "control_loop_queues",
    srcs = [
        "control_loops.q",
    ],
)

cc_library(
    name = "control_loop",
    srcs = [
        "control_loop.cc",
        "control_loop-tmpl.h",
    ],
    hdrs = [
        "control_loop.h",
    ],
    deps = [
        ":control_loop_queues",
        "//aos:queues",
        "//aos/events:event-loop",
        "//aos/events:shm-event-loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/time",
        "//aos/util:log_interval",
    ],
)
