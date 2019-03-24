filegroup(
    name = "binaries.stripped",
    srcs = [
        ":autonomous_action.stripped",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "binaries",
    srcs = [
        ":autonomous_action",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "autonomous_action_lib",
    srcs = [
        "auto_splines.cc",
        "autonomous_actor.cc",
    ],
    hdrs = [
        "auto_splines.h",
        "autonomous_actor.h",
    ],
    deps = [
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/control_loops/drivetrain:localizer_queue",
        "//y2019/control_loops/drivetrain:drivetrain_base",
        "//y2019/control_loops/superstructure:superstructure_queue",
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
        "//frc971/autonomous:auto_queue",
    ],
)
