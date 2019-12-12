licenses(["notice"])

cc_library(
    name = "matplotlib-cpp",
    hdrs = [
        "matplotlibcpp.h",
    ],
    restricted_to = ["//tools:k8"],
    visibility = ["//visibility:public"],
    deps = [
        "@python_repo//:python2.7_lib",
    ],
)

cc_binary(
    name = "basic_example",
    srcs = [
        "examples/basic.cpp",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "matplotlib-cpp",
    ],
)

cc_binary(
    name = "animation_example",
    srcs = [
        "examples/animation.cpp",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "matplotlib-cpp",
    ],
)
