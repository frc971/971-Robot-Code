load("//tools:platforms.bzl", "platforms")
load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "range_image",
    srcs = ["range_image.cc"],
    hdrs = ["range_image.h"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "contour",
    srcs = ["contour.cc"],
    hdrs = ["contour.h"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = platforms.any_of([
        "@platforms//cpu:x86_64",
        "//tools/platforms/hardware:raspberry_pi",
    ]),
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
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "find_blob",
    srcs = ["find_blob.cc"],
    hdrs = ["find_blob.h"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":codec",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "move_scale",
    srcs = ["move_scale.cc"],
    hdrs = ["move_scale.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":range_image",
        "//aos/vision/image:image_types",
    ],
)

cc_library(
    name = "test_utils",
    srcs = ["test_utils.cc"],
    hdrs = ["test_utils.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":range_image",
    ],
)

cc_library(
    name = "transpose",
    srcs = ["transpose.cc"],
    hdrs = ["transpose.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":range_image",
    ],
)

cc_test(
    name = "transpose_test",
    srcs = ["transpose_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":test_utils",
        ":transpose",
        "//aos/testing:googletest",
    ],
)

gtk_dependent_cc_library(
    name = "stream_view",
    hdrs = ["stream_view.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":range_image",
        "//aos/vision/debug:debug_window",
        "//aos/vision/image:image_types",
    ],
)
