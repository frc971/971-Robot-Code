genrule(
    name = "genrule_hood",
    outs = [
        "hood_plant.h",
        "hood_plant.cc",
        "hood_integral_plant.h",
        "hood_integral_plant.cc",
    ],
    cmd = "$(location //y2017/control_loops/python:hood) $(OUTS)",
    tools = [
        "//y2017/control_loops/python:hood",
    ],
)

cc_library(
    name = "hood_plants",
    srcs = [
        "hood_integral_plant.cc",
        "hood_plant.cc",
    ],
    hdrs = [
        "hood_integral_plant.h",
        "hood_plant.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "hood",
    srcs = [
        "hood.cc",
    ],
    hdrs = [
        "hood.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":hood_plants",
        "//frc971/control_loops:profiled_subsystem",
        "//y2017:constants",
        "//y2017/control_loops/superstructure:superstructure_goal_fbs",
        "//y2017/control_loops/superstructure:superstructure_position_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
    ],
)
