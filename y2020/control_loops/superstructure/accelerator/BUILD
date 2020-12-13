package(default_visibility = ["//visibility:public"])

genrule(
    name = "genrule_accelerator",
    outs = [
        "accelerator_plant.h",
        "accelerator_plant.cc",
        "integral_accelerator_plant.h",
        "integral_accelerator_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:accelerator) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:accelerator",
    ],
)

cc_library(
    name = "accelerator_plants",
    srcs = [
        "accelerator_plant.cc",
        "integral_accelerator_plant.cc",
    ],
    hdrs = [
        "accelerator_plant.h",
        "integral_accelerator_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
