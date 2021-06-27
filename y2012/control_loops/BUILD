load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2012:python_init"],
)

flatbuffer_cc_library(
    name = "control_loop_fbs",
    srcs = [
        "control_loops.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2012:__subpackages__"],
)
