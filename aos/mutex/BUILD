package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mutex",
    srcs = [
        "mutex.cc",
    ],
    hdrs = [
        "mutex.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/type_traits",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "mutex_test",
    srcs = [
        "mutex_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":mutex",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/testing:test_shm",
        "//aos/time",
        "//aos/util:death_test_log_implementation",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
