load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
        "//aos/mutex",
        "//aos/type_traits",
        "@com_github_google_glog//:glog",
    ],
)

# TODO(Brian): Remove this hack once bazel chases deps through selects correctly.
cc_library(
    name = "time_mcu",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
        "//aos/type_traits",
        "//motors/core",
    ],
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
