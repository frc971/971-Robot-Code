cc_library(
    name = "osqp-cpp",
    srcs = [
        "src/osqp++.cc",
    ],
    hdrs = [
        "include/osqp++.h",
    ],
    includes = [
        "include",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/osqp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "osqp-cpp_test",
    srcs = [
        "test/osqp++_test.cc",
    ],
    deps = [
        ":osqp-cpp",
        "//aos/testing:googletest",
        "@com_google_googletest//:gtest",
    ],
)
