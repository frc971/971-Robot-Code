package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "shooter_queue",
    srcs = [
        "shooter.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:queues",
    ],
)

genrule(
    name = "genrule_shooter",
    outs = [
        "shooter_plant.h",
        "shooter_plant.cc",
        "shooter_integral_plant.h",
        "shooter_integral_plant.cc",
    ],
    cmd = "$(location //y2016/control_loops/python:shooter) $(OUTS)",
    tools = [
        "//y2016/control_loops/python:shooter",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "shooter_plants",
    srcs = [
        "shooter_integral_plant.cc",
        "shooter_plant.cc",
    ],
    hdrs = [
        "shooter_integral_plant.h",
        "shooter_plant.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "shooter_lib",
    srcs = [
        "shooter.cc",
    ],
    hdrs = [
        "shooter.h",
    ],
    deps = [
        ":shooter_plants",
        ":shooter_queue",
        "//aos/controls:control_loop",
    ],
)

cc_test(
    name = "shooter_lib_test",
    srcs = [
        "shooter_lib_test.cc",
    ],
    deps = [
        ":shooter_lib",
        ":shooter_queue",
        "//aos:queues",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "shooter",
    srcs = [
        "shooter_main.cc",
    ],
    deps = [
        ":shooter_lib",
        ":shooter_queue",
        "//aos:init",
        "//aos/events:shm-event-loop",
    ],
)
