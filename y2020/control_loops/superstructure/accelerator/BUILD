package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

genrule(
    name = "genrule_accelerator",
    outs = [
        "accelerator_plant.h",
        "accelerator_plant.cc",
        "accelerator_integral_plant.h",
        "accelerator_integral_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:accelerator) $(OUTS)",
    tools = [
        "//y2020/control_loops/python:accelerator",
    ],
)
