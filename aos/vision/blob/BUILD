load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "range_image",
    srcs = ["range_image.cc"],
    hdrs = ["range_image.h"],
    deps = [
        "//aos/vision/debug:overlay",
        "//aos/vision/image:image_types",
        "//aos/vision/math:segment",
        "//aos/vision/math:vector",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "region_alloc",
    hdrs = ["region_alloc.h"],
)

cc_library(
    name = "contour",
    srcs = ["contour.cc"],
    hdrs = ["contour.h"],
    deps = [
        ":range_image",
        ":region_alloc",
        "//aos/vision/debug:overlay",
        "//aos/vision/math:segment",
    ],
)

cc_library(
    name = "threshold",
    srcs = [
        "threshold.cc",
    ],
    hdrs = [
        "threshold.h",
    ],
    deps = [
        ":range_image",
        "//aos/logging",
        "//aos/vision/image:image_types",
    ],
)

cc_test(
    name = "threshold_test",
    srcs = [
        "threshold_test.cc",
    ],
    deps = [
        ":range_image",
        ":threshold",
        "//aos/testing:googletest",
        "//aos/vision/image:image_types",
    ],
)

cc_library(
    name = "hierarchical_contour_merge",
    srcs = ["hierarchical_contour_merge.cc"],
    hdrs = ["hierarchical_contour_merge.h"],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    deps = [
        ":contour",
        ":disjoint_set",
        ":range_image",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "disjoint_set",
    hdrs = ["disjoint_set.h"],
)

cc_library(
    name = "find_blob",
    srcs = ["find_blob.cc"],
    hdrs = ["find_blob.h"],
    deps = [
        ":disjoint_set",
        ":range_image",
        "//aos/vision/debug:overlay",
        "//aos/vision/math:segment",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "codec",
    srcs = ["codec.cc"],
    hdrs = ["codec.h"],
    deps = [
        ":range_image",
        "//aos/vision/debug:overlay",
        "//aos/vision/math:segment",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "codec_test",
    srcs = ["codec_test.cc"],
    deps = [
        ":codec",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "move_scale",
    srcs = ["move_scale.cc"],
    hdrs = ["move_scale.h"],
    deps = [
        ":range_image",
        "//aos/vision/image:image_types",
    ],
)

cc_library(
    name = "test_utils",
    srcs = ["test_utils.cc"],
    hdrs = ["test_utils.h"],
    deps = [
        ":range_image",
    ],
)

cc_library(
    name = "transpose",
    srcs = ["transpose.cc"],
    hdrs = ["transpose.h"],
    deps = [
        ":range_image",
    ],
)

cc_test(
    name = "transpose_test",
    srcs = ["transpose_test.cc"],
    deps = [
        ":test_utils",
        ":transpose",
        "//aos/testing:googletest",
    ],
)

gtk_dependent_cc_library(
    name = "stream_view",
    hdrs = ["stream_view.h"],
    deps = [
        ":range_image",
        "//aos/vision/debug:debug_window",
        "//aos/vision/image:image_types",
    ],
)
