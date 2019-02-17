package(default_visibility = ["//visibility:public"])

load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    compatible_with = mcu_cpus,
    deps = [
        "//aos:macros",
        "//aos/type_traits",
    ] + select({
        # TODO(Brian): Deduplicate this (both CPU names and values).
        "//tools:cpu_cortex_m4f": [
            "//motors/core",
        ],
        "//tools:cpu_cortex_m4f_k22": [
            "//motors/core",
        ],
        "//conditions:default": [
            "//aos/ipc_lib:shared_mem",
            "//aos/logging",
            "//aos/mutex",
        ],
    }),
)

cc_test(
    name = "time_test",
    srcs = [
        "time_test.cc",
    ],
    deps = [
        ":time",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/util:death_test_log_implementation",
    ],
)
