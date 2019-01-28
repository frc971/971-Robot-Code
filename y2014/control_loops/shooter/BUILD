package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

cc_binary(
    name = "replay_shooter",
    srcs = [
        "replay_shooter.cc",
    ],
    deps = [
        ":shooter_queue",
        "//aos:init",
        "//aos/controls:replay_control_loop",
    ],
)

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
        "shooter_motor_plant.cc",
        "shooter_motor_plant.h",
        "unaugmented_shooter_motor_plant.cc",
        "unaugmented_shooter_motor_plant.h",
    ],
    cmd = "$(location //y2014/control_loops/python:shooter) $(OUTS)",
    tools = [
        "//y2014/control_loops/python:shooter",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "shooter_lib",
    srcs = [
        "shooter.cc",
        "shooter_motor_plant.cc",
        "unaugmented_shooter_motor_plant.cc",
    ],
    hdrs = [
        "shooter.h",
        "shooter_motor_plant.h",
        "unaugmented_shooter_motor_plant.h",
    ],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":shooter_queue",
        "//aos/controls:control_loop",
        "//aos/logging:queue_logging",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014:constants",
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
        "//aos:init",
        "//aos/events:shm-event-loop",
    ],
)
