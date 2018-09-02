package(default_visibility = ["//visibility:public"])

cc_library(
    name = "segment",
    hdrs = ["segment.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [":vector"],
)

cc_library(
    name = "vector",
    hdrs = [
        "vector.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//third_party/eigen",
    ],
)

cc_test(
    name = "vector_test",
    size = "small",
    srcs = [
        "vector_test.cc",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":vector",
        "//aos/testing:googletest",
    ],
)
