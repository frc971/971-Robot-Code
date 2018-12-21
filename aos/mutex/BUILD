package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mutex",
    srcs = [
        "mutex.cc",
    ],
    hdrs = [
        "mutex.h",
    ],
    deps = [
        "//aos:die",
        "//aos/type_traits:type_traits",
        "//aos/logging",
        "//aos/ipc_lib:aos_sync",
    ],
)

cc_test(
    name = "mutex_test",
    srcs = [
        "mutex_test.cc",
    ],
    deps = [
        "//aos:die",
        ":mutex",
        "//aos/time:time",
        "//aos/logging",
        "//aos/util:death_test_log_implementation",
        "//aos/util:thread",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/testing:test_shm",
    ],
)
