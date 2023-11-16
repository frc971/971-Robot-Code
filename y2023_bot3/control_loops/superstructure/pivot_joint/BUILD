package(default_visibility = ["//y2023_bot3:__subpackages__"])

genrule(
    name = "genrule_pivot_joint",
    outs = [
        "pivot_joint_plant.h",
        "pivot_joint_plant.cc",
        "integral_pivot_joint_plant.h",
        "integral_pivot_joint_plant.cc",
    ],
    cmd = "$(location //y2023_bot3/control_loops/python:pivot_joint) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2023_bot3/control_loops/python:pivot_joint",
    ],
)

cc_library(
    name = "pivot_joint_plants",
    srcs = [
        "integral_pivot_joint_plant.cc",
        "pivot_joint_plant.cc",
    ],
    hdrs = [
        "integral_pivot_joint_plant.h",
        "pivot_joint_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
