package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stl_mutex",
    hdrs = [
        "stl_mutex.h",
    ],
    deps = [
        "//aos/logging",
        "//aos/ipc_lib:aos_sync",
    ],
)

cc_test(
    name = "stl_mutex_test",
    srcs = [
        "stl_mutex_test.cc",
    ],
    deps = [
        "//aos:die",
        ":stl_mutex",
        "//aos/util:thread",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)
