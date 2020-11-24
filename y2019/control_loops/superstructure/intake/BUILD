package(default_visibility = ["//y2019:__subpackages__"])

genrule(
    name = "genrule_intake",
    outs = [
        "intake_plant.h",
        "intake_plant.cc",
        "integral_intake_plant.h",
        "integral_intake_plant.cc",
    ],
    cmd = "$(location //y2019/control_loops/python:intake) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2019/control_loops/python:intake",
    ],
)

cc_library(
    name = "intake_plants",
    srcs = [
        "intake_plant.cc",
        "integral_intake_plant.cc",
    ],
    hdrs = [
        "intake_plant.h",
        "integral_intake_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
