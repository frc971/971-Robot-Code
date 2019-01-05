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
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2018/control_loops/drivetrain:drivetrain_base",
        "//y2018/control_loops/superstructure:superstructure_queue",
        "//y2018/control_loops/superstructure/arm:generated_graph",
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
        "//frc971/autonomous:auto_queue",
    ],
)
