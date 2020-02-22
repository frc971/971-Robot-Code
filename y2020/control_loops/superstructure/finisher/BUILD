package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

genrule(
    name = "genrule_finisher",
    outs = [
        "finisher_plant.h",
        "finisher_plant.cc",
        "integral_finisher_plant.h",
        "integral_finisher_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:finisher) $(OUTS)",
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
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
