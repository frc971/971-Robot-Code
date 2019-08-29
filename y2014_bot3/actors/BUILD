cc_library(
    name = "autonomous_action_lib",
    srcs = [
        "autonomous_actor.cc",
    ],
    hdrs = [
        "autonomous_actor.h",
    ],
    deps = [
        "//aos/actions:action_lib",
        "//aos/events:event_loop",
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//y2014_bot3/control_loops/drivetrain:drivetrain_base",
    ],
)

cc_binary(
    name = "autonomous_action",
    srcs = [
        "autonomous_actor_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_action_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
