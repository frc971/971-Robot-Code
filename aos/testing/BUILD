cc_library(
    name = "googletest",
    testonly = True,
    srcs = [
        "gtest_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "test_logging",
    testonly = True,
    srcs = [
        "test_logging.cc",
    ],
    hdrs = [
        "test_logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":googletest",
        "//aos:once",
        "//aos/logging:implementations",
        "//aos/mutex",
    ],
)

cc_test(
    name = "test_logging_test",
    srcs = [
        "test_logging_test.cc",
    ],
    deps = [
        ":googletest",
        ":test_logging",
        "//aos/logging",
    ],
)

cc_library(
    name = "prevent_exit",
    testonly = True,
    srcs = [
        "prevent_exit.cc",
    ],
    hdrs = [
        "prevent_exit.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "test_shm",
    testonly = True,
    srcs = [
        "test_shm.cc",
    ],
    hdrs = [
        "test_shm.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":test_logging",
        "//aos/ipc_lib:queue",
        "//aos/ipc_lib:shared_mem",
        "//aos/logging",
    ],
)

cc_library(
    name = "random_seed",
    testonly = True,
    srcs = [
        "random_seed.cc",
    ],
    hdrs = [
        "random_seed.h",
    ],
    visibility = ["//visibility:public"],
)
