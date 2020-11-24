licenses(["notice"])

cc_library(
    name = "matplotlib-cpp",
    hdrs = [
        "matplotlibcpp.h",
    ],
    data = [
        "@matplotlib_repo//:matplotlib3",
        "@python_repo//:all_files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@python_repo//:python3.5_lib",
    ],
)

cc_binary(
    name = "basic_example",
    srcs = [
        "examples/basic.cpp",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "matplotlib-cpp",
    ],
)

cc_binary(
    name = "animation_example",
    srcs = [
        "examples/animation.cpp",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "matplotlib-cpp",
    ],
)
