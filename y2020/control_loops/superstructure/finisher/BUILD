package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

genrule(
    name = "genrule_finisher",
    outs = [
        "finisher_plant.h",
        "finisher_plant.cc",
        "finisher_integral_plant.h",
        "finisher_integral_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:finisher) $(OUTS)",
    tools = [
        "//y2020/control_loops/python:finisher",
    ],
)
