package(default_visibility = ["//y2019:__subpackages__"])

genrule(
    name = "genrule_elevator",
    outs = [
        "elevator_plant.h",
        "elevator_plant.cc",
        "integral_elevator_plant.h",
        "integral_elevator_plant.cc",
    ],
    cmd = "$(location //y2019/control_loops/python:elevator) $(OUTS)",
    tools = [
        "//y2019/control_loops/python:elevator",
    ],
)

cc_library(
    name = "elevator_plants",
    srcs = [
        "elevator_plant.cc",
        "integral_elevator_plant.cc",
    ],
    hdrs = [
        "elevator_plant.h",
        "integral_elevator_plant.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
