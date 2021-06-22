package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_binary(
    name = "accessories",
    srcs = [
        "accessories.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":accessories_fbs",
        "//aos:init",
        "//frc971/control_loops:control_loop",
        "//y2012/control_loops:control_loop_fbs",
    ],
)

flatbuffer_cc_library(
    name = "accessories_fbs",
    srcs = [
        "accessories.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
