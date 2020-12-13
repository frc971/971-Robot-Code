package(default_visibility = ["//visibility:public"])

genrule(
    name = "genrule_finisher",
    outs = [
        "finisher_plant.h",
        "finisher_plant.cc",
        "integral_finisher_plant.h",
        "integral_finisher_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:finisher) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:finisher",
    ],
)

cc_library(
    name = "finisher_plants",
    srcs = [
        "finisher_plant.cc",
        "integral_finisher_plant.cc",
    ],
    hdrs = [
        "finisher_plant.h",
        "integral_finisher_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
