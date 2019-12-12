package(default_visibility = ["//y2019:__subpackages__"])

genrule(
    name = "genrule_stilts",
    outs = [
        "stilts_plant.h",
        "stilts_plant.cc",
        "integral_stilts_plant.h",
        "integral_stilts_plant.cc",
    ],
    cmd = "$(location //y2019/control_loops/python:stilts) $(OUTS)",
    tools = [
        "//y2019/control_loops/python:stilts",
    ],
)

cc_library(
    name = "stilts_plants",
    srcs = [
        "integral_stilts_plant.cc",
        "stilts_plant.cc",
    ],
    hdrs = [
        "integral_stilts_plant.h",
        "stilts_plant.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
