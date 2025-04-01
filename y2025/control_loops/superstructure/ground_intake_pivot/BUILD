package(default_visibility = ["//y2025:__subpackages__"])

genrule(
    name = "genrule_ground_intake_pivot",
    outs = [
        "ground_intake_pivot_plant.h",
        "ground_intake_pivot_plant.cc",
        "ground_intake_pivot_plant.json",
        "integral_ground_intake_pivot_plant.h",
        "integral_ground_intake_pivot_plant.cc",
        "integral_ground_intake_pivot_plant.json",
    ],
    cmd = "$(location //y2025/control_loops/python:ground_intake_pivot) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2025/control_loops/python:ground_intake_pivot",
    ],
)

cc_library(
    name = "ground_intake_pivot_plants",
    srcs = [
        "ground_intake_pivot_plant.cc",
        "integral_ground_intake_pivot_plant.cc",
    ],
    hdrs = [
        "ground_intake_pivot_plant.h",
        "integral_ground_intake_pivot_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "ground_intake_pivot_json",
    srcs = ["integral_ground_intake_pivot_plant.json"],
    visibility = ["//visibility:public"],
)
