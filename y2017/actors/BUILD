package(default_visibility = ["//visibility:public"])

filegroup(
    name = "binaries",
    srcs = [
        ":autonomous_action",
    ],
)

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
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//y2017/control_loops/drivetrain:drivetrain_base",
        "//y2017/control_loops/superstructure:superstructure_goal_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_binary(
    name = "autonomous_action",
    srcs = [
        "autonomous_actor_main.cc",
    ],
    deps = [
        ":autonomous_action_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
