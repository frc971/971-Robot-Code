package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stl_mutex",
    hdrs = [
        "stl_mutex.h",
    ],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/logging",
    ],
)

cc_test(
    name = "stl_mutex_test",
    srcs = [
        "stl_mutex_test.cc",
    ],
    deps = [
        ":stl_mutex",
        "//aos:die",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/util:thread",
    ],
)
