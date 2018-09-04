licenses(["notice"])

load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "GSL",
    hdrs = glob(["include/**"]),
    compatible_with = mcu_cpus,
    includes = ["include"],
    visibility = ["//visibility:public"],
)
