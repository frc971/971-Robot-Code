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
        "//aos/common:queues",
        "//aos/common:time",
        "//aos/common/logging:queue_logging",
        "//aos/common/logging:replay",
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
        "//aos/common:time",
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
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
        "//aos/common/logging",
        "//aos/common/logging:matrix_logging",
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
        "//third_party/googletest:googlemock",
    ],
)

queue_library(
    name = "control_loop_queues",
    srcs = [
        "control_loops.q",
    ],
    compatible_with = [
        "//tools:armhf-debian",
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
        "//aos/common:queues",
        "//aos/common:time",
        "//aos/common/logging",
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
        "//aos/common/util:log_interval",
    ],
)
