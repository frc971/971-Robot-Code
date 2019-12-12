package(default_visibility = ["//y2019:__subpackages__"])

genrule(
    name = "genrule_wrist",
    outs = [
        "wrist_plant.h",
        "wrist_plant.cc",
        "integral_wrist_plant.h",
        "integral_wrist_plant.cc",
    ],
    cmd = "$(location //y2019/control_loops/python:wrist) $(OUTS)",
    tools = [
        "//y2019/control_loops/python:wrist",
    ],
)

cc_library(
    name = "wrist_plants",
    srcs = [
        "integral_wrist_plant.cc",
        "wrist_plant.cc",
    ],
    hdrs = [
        "integral_wrist_plant.h",
        "wrist_plant.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
