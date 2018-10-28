package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos:macros",
        "//aos/mutex:mutex",
        "//aos/logging",
        "//aos/ipc_lib:shared_mem",
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
        "//aos/util:death_test_log_implementation",
        "//aos/testing:googletest",
    ],
)
