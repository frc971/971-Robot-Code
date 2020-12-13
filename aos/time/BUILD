cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "@platforms//os:linux": ["@com_github_google_glog//:glog"],
        "//conditions:default": ["//motors/core"],
    }),
)

cc_test(
    name = "time_test",
    srcs = [
        "time_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":time",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/util:death_test_log_implementation",
    ],
)
