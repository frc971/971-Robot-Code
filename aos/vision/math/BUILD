package(default_visibility = ["//visibility:public"])

cc_library(
    name = "segment",
    hdrs = ["segment.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":vector"],
)

cc_library(
    name = "vector",
    hdrs = [
        "vector.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "vector_test",
    size = "small",
    srcs = [
        "vector_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":vector",
        "//aos/testing:googletest",
    ],
)
