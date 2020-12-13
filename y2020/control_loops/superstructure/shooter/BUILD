package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shooter_plants",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "shooter",
    srcs = [
        "shooter.cc",
    ],
    hdrs = [
        "shooter.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flywheel_controller",
        "//aos/controls:control_loop",
        "//frc971/control_loops:profiled_subsystem",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_library(
    name = "flywheel_controller",
    srcs = [
        "flywheel_controller.cc",
    ],
    hdrs = [
        "flywheel_controller.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/controls:control_loop",
        "//frc971/control_loops:profiled_subsystem",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
        "//y2020/control_loops/superstructure/accelerator:accelerator_plants",
        "//y2020/control_loops/superstructure/finisher:finisher_plants",
    ],
)
