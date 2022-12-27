licenses(["notice"])

cc_library(
    name = "matplotlib-cpp",
    hdrs = [
        "matplotlibcpp.h",
    ],
    data = select({
        "//tools/platforms/python:debian_bundled_python": [
            "@matplotlib_repo//:matplotlib3",
        ],
        "//tools/platforms/python:upstream_bundled_python": [
            "@pip//matplotlib",
            "@pip//pygobject",
        ],
    }) + [
        "//third_party/python:python_runtime",
    ],
    # While this is technically compatible with "linux", the
    # "@python_repo//:all_files" has x86 binaries in it.
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = select({
        "//tools/platforms/python:debian_bundled_python": [],
        "//tools/platforms/python:upstream_bundled_python": [
            "//third_party/python:numpy_cc",
        ],
    }) + [
        "//third_party/python",
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
