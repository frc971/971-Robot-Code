load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "matrix_fbs",
    srcs = ["matrix.fbs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "flatbuffers_matrix",
    hdrs = ["flatbuffers_matrix.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":matrix_fbs",
        "//aos:json_to_flatbuffer",
        "@com_github_google_glog//:glog",
        "@com_github_tartanllama_expected",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "flatbuffers_matrix_test",
    srcs = ["flatbuffers_matrix_test.cc"],
    deps = [
        ":flatbuffers_matrix",
        "//aos/testing:googletest",
    ],
)
