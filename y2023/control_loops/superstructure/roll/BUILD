package(default_visibility = ["//y2023:__subpackages__"])

genrule(
    name = "genrule_roll",
    outs = [
        "roll_plant.h",
        "roll_plant.cc",
        "integral_roll_plant.h",
        "integral_roll_plant.cc",
    ],
    cmd = "$(location //y2023/control_loops/python:roll) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2023/control_loops/python:roll",
    ],
)

genrule(
    name = "genrule_hybrid_roll",
    outs = [
        "hybrid_roll_plant.h",
        "hybrid_roll_plant.cc",
        "integral_hybrid_roll_plant.h",
        "integral_hybrid_roll_plant.cc",
    ],
    cmd = "$(location //y2023/control_loops/python:roll) --hybrid $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2023/control_loops/python:roll",
    ],
)

cc_library(
    name = "roll_plants",
    srcs = [
        "hybrid_roll_plant.cc",
        "integral_hybrid_roll_plant.cc",
        "integral_roll_plant.cc",
        "roll_plant.cc",
    ],
    hdrs = [
        "hybrid_roll_plant.h",
        "integral_hybrid_roll_plant.h",
        "integral_roll_plant.h",
        "roll_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
