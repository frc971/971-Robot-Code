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
        "@com_github_google_glog//:glog",
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
        "@com_github_google_glog//:glog",
    ],
)
