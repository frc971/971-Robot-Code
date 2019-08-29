filegroup(
    name = "binaries",
    srcs = [
        ":autonomous_action",
        ":shoot_action",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "binaries.stripped",
    srcs = [
        ":autonomous_action.stripped",
        ":shoot_action.stripped",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "shoot_action_lib",
    srcs = [
        "shoot_actor.cc",
    ],
    hdrs = [
        "shoot_actor.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/actions:action_lib",
        "//aos/logging",
        "//y2014:constants",
        "//y2014/control_loops/claw:claw_goal_fbs",
        "//y2014/control_loops/claw:claw_status_fbs",
        "//y2014/control_loops/shooter:shooter_goal_fbs",
        "//y2014/control_loops/shooter:shooter_status_fbs",
    ],
)

cc_binary(
    name = "shoot_action",
    srcs = [
        "shoot_actor_main.cc",
    ],
    deps = [
        ":shoot_action_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
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
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//y2014/actors:shoot_action_lib",
        "//y2014/control_loops/claw:claw_goal_fbs",
        "//y2014/control_loops/claw:claw_status_fbs",
        "//y2014/control_loops/drivetrain:drivetrain_base",
        "//y2014/control_loops/shooter:shooter_goal_fbs",
        "//y2014/queues:auto_mode_fbs",
        "//y2014/queues:hot_goal_fbs",
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
