package(default_visibility = ["//y2024_bot3:__subpackages__"])

genrule(
    name = "genrule_arm",
    outs = [
        "arm_plant.h",
        "arm_plant.cc",
        "arm_plant.json",
        "integral_arm_plant.h",
        "integral_arm_plant.cc",
        "integral_arm_plant.json",
    ],
    cmd = "$(location //y2024_bot3/control_loops/python:arm) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024_bot3/control_loops/python:arm",
    ],
)

cc_library(
    name = "arm_plants",
    srcs = [
        "arm_plant.cc",
        "integral_arm_plant.cc",
    ],
    hdrs = [
        "arm_plant.h",
        "integral_arm_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "arm_json",
    srcs = ["integral_arm_plant.json"],
    visibility = ["//visibility:public"],
)
