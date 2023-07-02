package(default_visibility = ["//y2023:__subpackages__"])

genrule(
    name = "genrule_turret",
    outs = [
        "turret_plant.h",
        "turret_plant.cc",
        "integral_turret_plant.h",
        "integral_turret_plant.cc",
    ],
    cmd = "$(location //y2023/control_loops/python:turret) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2023/control_loops/python:turret",
    ],
)

cc_library(
    name = "turret_plants",
    srcs = [
        "integral_turret_plant.cc",
        "turret_plant.cc",
    ],
    hdrs = [
        "integral_turret_plant.h",
        "turret_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
