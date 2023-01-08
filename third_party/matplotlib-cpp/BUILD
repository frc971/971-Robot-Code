licenses(["notice"])

cc_library(
    name = "matplotlib-cpp",
    hdrs = [
        "matplotlibcpp.h",
    ],
    data = [
        "//third_party/python:python_runtime",
        "@pip//matplotlib",
        "@pip//pygobject",
    ],
    # While this is technically compatible with "linux", the
    # "@python_repo//:all_files" has x86 binaries in it.
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/python",
        "//third_party/python:numpy_cc",
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
