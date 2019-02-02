package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "superstructure_queue",
    srcs = [
        "superstructure.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:profiled_subsystem_queue",
        "//frc971/control_loops:queues",
    ],
)

cc_library(
    name = "superstructure_lib",
    srcs = [
        "superstructure.cc",
    ],
    hdrs = [
        "superstructure.h",
    ],
    deps = [
        ":superstructure_queue",
        "//aos/controls:control_loop",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
    deps = [
        ":superstructure_lib",
        "//aos:init",
        "//aos/events:shm-event-loop",
    ],
)
