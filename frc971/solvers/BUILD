cc_library(
    name = "convex",
    hdrs = ["convex.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "convex_test",
    srcs = [
        "convex_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":convex",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "sparse_convex",
    srcs = ["sparse_convex.cc"],
    hdrs = ["sparse_convex.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "sparse_convex_test",
    srcs = ["sparse_convex_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sparse_convex",
        "//aos/testing:googletest",
    ],
)
