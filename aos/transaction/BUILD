package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transaction",
    hdrs = [
        "transaction.h",
    ],
    deps = [
        "//aos/logging",
        "//aos/util:compiler_memory_barrier",
    ],
)

cc_test(
    name = "transaction_test",
    srcs = [
        "transaction_test.cc",
    ],
    deps = [
        ":transaction",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/util:death_test_log_implementation",
    ],
)
