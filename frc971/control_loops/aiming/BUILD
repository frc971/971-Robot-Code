cc_library(
    name = "aiming",
    srcs = ["aiming.cc"],
    hdrs = ["aiming.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/logging",
        "//frc971:constants",
        "//frc971/control_loops:pose",
    ],
)

cc_test(
    name = "aiming_test",
    srcs = ["aiming_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aiming",
        "//aos/testing:googletest",
    ],
)
