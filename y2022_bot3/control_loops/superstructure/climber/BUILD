package(default_visibility = ["//y2022_bot3:__subpackages__"])

genrule(
    name = "genrule_climber",
    outs = [
        "climber_plant.h",
        "climber_plant.cc",
        "integral_climber_plant.h",
        "integral_climber_plant.cc",
    ],
    cmd = "$(location //y2022_bot3/control_loops/python:climber) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2022_bot3/control_loops/python:climber",
    ],
)

cc_library(
    name = "climber_plants",
    srcs = [
        "climber_plant.cc",
        "integral_climber_plant.cc",
    ],
    hdrs = [
        "climber_plant.h",
        "integral_climber_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
