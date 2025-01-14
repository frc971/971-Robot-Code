package(default_visibility = ["//y2025:__subpackages__"])

genrule(
    name = "genrule_pivot",
    outs = [
        "pivot_plant.h",
        "pivot_plant.cc",
        "pivot_plant.json",
        "integral_pivot_plant.h",
        "integral_pivot_plant.cc",
        "integral_pivot_plant.json",
    ],
    cmd = "$(location //y2025/control_loops/python:pivot) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2025/control_loops/python:pivot",
    ],
)

cc_library(
    name = "pivot_plants",
    srcs = [
        "integral_pivot_plant.cc",
        "pivot_plant.cc",
    ],
    hdrs = [
        "integral_pivot_plant.h",
        "pivot_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "pivot_json",
    srcs = ["integral_pivot_plant.json"],
    visibility = ["//visibility:public"],
)
