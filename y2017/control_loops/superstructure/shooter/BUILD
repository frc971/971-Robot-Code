package(default_visibility = ["//visibility:public"])

genrule(
    name = "genrule_shooter",
    outs = [
        "shooter_plant.h",
        "shooter_plant.cc",
        "shooter_integral_plant.h",
        "shooter_integral_plant.cc",
    ],
    cmd = "$(location //y2017/control_loops/python:shooter) $(OUTS)",
    tools = [
        "//y2017/control_loops/python:shooter",
    ],
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
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
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
    visibility = ["//visibility:public"],
    deps = [
        ":shooter_plants",
        "//aos/controls:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//y2017/control_loops/superstructure:superstructure_goal_fbs",
        "//y2017/control_loops/superstructure:superstructure_position_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
