package(default_visibility = ["//visibility:public"])

load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "type_traits",
    hdrs = [
        "type_traits.h",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "type_traits_test",
    srcs = [
        "type_traits_test.cpp",
    ],
    deps = [
        ":type_traits",
        "//aos/testing:googletest",
    ],
)
