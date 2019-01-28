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
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/util:trapezoid_profile",
        "//frc971/control_loops:profiled_subsystem",
        "//frc971/control_loops:simple_capped_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/zeroing",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
    deps = [
        ":superstructure_lib",
        ":superstructure_queue",
        "//aos:init",
        "//aos/events:shm-event-loop",
    ],
)
