cc_library(
    name = "convex",
    hdrs = ["convex.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
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
