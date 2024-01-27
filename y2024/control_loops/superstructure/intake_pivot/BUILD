package(default_visibility = ["//y2024:__subpackages__"])

genrule(
    name = "genrule_intake_pivot",
    outs = [
        "intake_pivot_plant.h",
        "intake_pivot_plant.cc",
        "intake_pivot_plant.json",
        "integral_intake_pivot_plant.h",
        "integral_intake_pivot_plant.cc",
        "integral_intake_pivot_plant.json",
    ],
    cmd = "$(location //y2024/control_loops/python:intake_pivot) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024/control_loops/python:intake_pivot",
    ],
)

cc_library(
    name = "intake_pivot_plants",
    srcs = [
        "intake_pivot_plant.cc",
        "integral_intake_pivot_plant.cc",
    ],
    hdrs = [
        "intake_pivot_plant.h",
        "integral_intake_pivot_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "intake_pivot_json",
    srcs = ["integral_intake_pivot_plant.json"],
    visibility = ["//visibility:public"],
)
