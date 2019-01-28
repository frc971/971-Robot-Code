package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "rollers_queue",
    srcs = [
        "rollers.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:queues",
    ],
)

cc_library(
    name = "rollers_lib",
    srcs = [
        "rollers.cc",
    ],
    hdrs = [
        "rollers.h",
    ],
    deps = [
        ":rollers_queue",
        "//aos/controls:control_loop",
        "//aos/logging",
    ],
)

cc_binary(
    name = "rollers",
    srcs = [
        "rollers_main.cc",
    ],
    deps = [
        ":rollers_lib",
        "//aos:init",
        "//aos/events:shm-event-loop",
    ],
)
