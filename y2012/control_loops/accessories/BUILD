package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_binary(
    name = "accessories",
    srcs = [
        "accessories.cc",
    ],
    deps = [
        ":accessories_fbs",
        "//aos:init",
        "//aos/controls:control_loop",
        "//aos/controls:control_loop_fbs",
    ],
)

flatbuffer_cc_library(
    name = "accessories_fbs",
    srcs = [
        "accessories.fbs",
    ],
)
