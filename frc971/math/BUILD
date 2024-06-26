load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "matrix_fbs",
    srcs = ["matrix.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "matrix_ts_fbs",
    srcs = [
        "matrix.fbs",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "flatbuffers_matrix",
    hdrs = ["flatbuffers_matrix.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":matrix_fbs",
        "//aos:json_to_flatbuffer",
        "@com_github_tartanllama_expected",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
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

cc_library(
    name = "interpolate",
    hdrs = ["interpolate.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "interpolate_test",
    srcs = ["interpolate_test.cc"],
    deps = [
        ":interpolate",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
