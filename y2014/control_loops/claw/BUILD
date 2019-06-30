package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "claw_queue",
    srcs = [
        "claw.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:queues",
    ],
)

genrule(
    name = "genrule_claw",
    outs = [
        "claw_motor_plant.h",
        "claw_motor_plant.cc",
    ],
    cmd = "$(location //y2014/control_loops/python:claw) $(OUTS)",
    tools = [
        "//y2014/control_loops/python:claw",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "claw_lib",
    srcs = [
        "claw.cc",
        "claw_motor_plant.cc",
    ],
    hdrs = [
        "claw.h",
        "claw_motor_plant.h",
    ],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":claw_queue",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/controls:polytope",
        "//aos/logging:matrix_logging",
        "//aos/logging:queue_logging",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:hall_effect_tracker",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014:constants",
    ],
)

cc_test(
    name = "claw_lib_test",
    srcs = [
        "claw_lib_test.cc",
    ],
    deps = [
        ":claw_lib",
        ":claw_queue",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "claw",
    srcs = [
        "claw_main.cc",
    ],
    deps = [
        ":claw_lib",
        "//aos:init",
        "//aos/events:shm-event-loop",
    ],
)
