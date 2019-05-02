package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "auto_queue",
    srcs = [
        "auto.q",
    ],
    deps = [
        "//aos/actions:action_queue",
    ],
)

cc_library(
    name = "base_autonomous_actor",
    srcs = [
        "base_autonomous_actor.cc",
    ],
    hdrs = [
        "base_autonomous_actor.h",
    ],
    deps = [
        ":auto_queue",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/control_loops/drivetrain:localizer_queue",
        "//y2019/control_loops/drivetrain:target_selector_queue",
    ],
)
