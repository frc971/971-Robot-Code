cc_library(
    name = "interpolation",
    srcs = [
        "interpolation.cc",
    ],
    hdrs = [
        "interpolation.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "interpolation_test",
    srcs = [
        "interpolation_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":interpolation",
        "//aos/testing:googletest",
    ],
)
