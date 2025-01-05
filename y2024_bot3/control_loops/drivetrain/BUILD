genrule(
    name = "genrule_rotation",
    outs = [
        "rotation_plant.h",
        "rotation_plant.cc",
        "rotation_plant.json",
        "integral_rotation_plant.h",
        "integral_rotation_plant.cc",
        "integral_rotation_plant.json",
    ],
    cmd = "$(location //y2024_bot3/control_loops/python:rotation) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024_bot3/control_loops/python:rotation",
    ],
)

cc_library(
    name = "rotation_plants",
    srcs = [
        "integral_rotation_plant.cc",
        "rotation_plant.cc",
    ],
    hdrs = [
        "integral_rotation_plant.h",
        "rotation_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "rotation_json",
    srcs = ["integral_rotation_plant.json"],
    visibility = ["//visibility:public"],
)
