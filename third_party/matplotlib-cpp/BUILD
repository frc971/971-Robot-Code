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
    # While this is technically compatible with "linux", the
    # "@python_repo//:all_files" has x86 binaries in it.
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "@python_repo//:python3.9_lib",
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
