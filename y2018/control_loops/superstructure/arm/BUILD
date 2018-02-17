cc_library(
    name = "trajectory",
    srcs = [
        "trajectory.cc",
    ],
    hdrs = [
        "trajectory.h",
    ],
    deps = ["//third_party/eigen"],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    deps = [
        ":trajectory",
        "//aos/testing:googletest",
        "//third_party/eigen",
    ],
)
