cc_library(
    name = "googletest",
    testonly = True,
    srcs = [
        "gtest_main.cc",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/googletest:gtest",
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
        "//aos/mutex:mutex",
        "//aos/logging:implementations",
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
        "//aos/logging",
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
        "//aos:queues",
        "//aos/logging",
        "//aos/ipc_lib:shared_mem",
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
