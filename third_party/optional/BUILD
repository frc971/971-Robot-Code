licenses(["notice"])

load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "optional",
    hdrs = [
        "tl/optional.hpp",
    ],
    compatible_with = mcu_cpus,
    includes = ["."],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "catch",
    hdrs = [
        "tests/catch.hpp",
    ],
)

# Create a small wrapper because the tests want to #include "optional.hpp"
# directly.
cc_library(
    name = "optional_test_wrapper",
    includes = [
        "./tl/",
    ],
    deps = [
        ":optional",
    ],
)

cc_test(
    name = "optional_test",
    srcs = glob([
        "tests/*.cpp",
    ]),
    copts = [
        "-fexceptions",
        "-Wno-unused-parameter",
    ],
    deps = [
        ":catch",
        ":optional_test_wrapper",
    ],
)
