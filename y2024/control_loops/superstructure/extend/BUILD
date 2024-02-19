package(default_visibility = ["//y2024:__subpackages__"])

genrule(
    name = "genrule_extend",
    outs = [
        "extend_plant.h",
        "extend_plant.cc",
        "extend_plant.json",
        "integral_extend_plant.h",
        "integral_extend_plant.cc",
        "integral_extend_plant.json",
    ],
    cmd = "$(location //y2024/control_loops/python:extend) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024/control_loops/python:extend",
    ],
)

cc_library(
    name = "extend_plants",
    srcs = [
        "extend_plant.cc",
        "integral_extend_plant.cc",
    ],
    hdrs = [
        "extend_plant.h",
        "integral_extend_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "extend_json",
    srcs = ["integral_extend_plant.json"],
    visibility = ["//visibility:public"],
)
