package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

genrule(
    name = "genrule_accelerator",
    outs = [
        "accelerator_plant.h",
        "accelerator_plant.cc",
        "integral_accelerator_plant.h",
        "integral_accelerator_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:accelerator) $(OUTS)",
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
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
