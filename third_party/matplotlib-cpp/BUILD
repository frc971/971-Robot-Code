licenses(["notice"])

cc_library(
    name = "matplotlib-cpp",
    hdrs = [
        "matplotlibcpp.h"
    ],
    deps = [
        "@usr_repo//:python2.7_lib",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:k8"],
)

cc_binary(
    name = "basic_example",
    srcs = [
        "examples/basic.cpp",
    ],
    deps = [
        "matplotlib-cpp",
    ],
    restricted_to = ["//tools:k8"],
)

cc_binary(
    name = "animation_example",
    srcs = [
        "examples/animation.cpp",
    ],
    deps = [
        "matplotlib-cpp",
    ],
    restricted_to = ["//tools:k8"],
)
