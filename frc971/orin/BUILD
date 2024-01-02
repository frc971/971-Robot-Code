load("//frc971:halide.bzl", "halide_library")

halide_library(
    name = "ycbcr",
    src = "crcv_generator.cc",
    args = "rows=1088 cols=1456 ystride=2048 cbcrstride=3840",
    function = "ycbcr",
    visibility = ["//visibility:public"],
)

halide_library(
    name = "ycbcr422",
    src = "crcv_generator.cc",
    args = "rows=1088 cols=1456 ystride=1536 cbcrstride=1536",
    function = "ycbcr422",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "apriltag",
    srcs = [
        "apriltag.cc",
        "apriltag_detect.cc",
        "cuda.cc",
        "labeling_allegretti_2019_BKE.cc",
        "line_fit_filter.cc",
        "points.cc",
        "threshold.cc",
    ],
    hdrs = [
        "apriltag.h",
        "cuda.h",
        "gpu_image.h",
        "labeling_allegretti_2019_BKE.h",
        "line_fit_filter.h",
        "points.h",
        "threshold.h",
        "transform_output_iterator.h",
    ],
    copts = [
        "-Wno-pass-failed",
        #"-DCUB_DETAIL_DEBUG_ENABLE_LOG=1",
        #"-DDEBUG=1",
    ],
    features = ["cuda"],
    deps = [
        "//aos/time",
        "//third_party:cudart",
        "//third_party/apriltag",
        "@com_github_google_glog//:glog",
        "@com_github_nvidia_cccl//:cccl",
        "@com_github_nvidia_cuco//:cuco",
    ],
)

cc_test(
    name = "cuda_april_tag_test",
    srcs = [
        "cuda_april_tag_test.cc",
    ],
    data = [
        "@apriltag_test_bfbs_images",
        "@orin_image_apriltag//file",
        "@orin_large_image_apriltag//file",
    ],
    features = ["cuda"],
    deps = [
        ":apriltag",
        ":ycbcr",
        "//aos:flatbuffer_merge",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//third_party/apriltag",
        "//y2023/vision:ThresholdHalide",
        "//y2023/vision:ToGreyscaleAndDecimateHalide",
    ],
)

cc_test(
    name = "output_iterator_test",
    srcs = [
        "output_iterator_test.cc",
    ],
    features = ["cuda"],
    deps = [
        ":apriltag",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
    ],
)

cc_test(
    name = "points_test",
    srcs = [
        "points_test.cc",
    ],
    features = ["cuda"],
    deps = [
        ":apriltag",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
    ],
)

cc_binary(
    name = "argus_camera",
    srcs = [
        "argus_camera.cc",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/util:file",
        "//frc971/orin:ycbcr",
        "//frc971/orin:ycbcr422",
        "//frc971/vision:vision_fbs",
        "@arm64_debian_sysroot//:argus",
        "@arm64_debian_sysroot//:egl",
        "@arm64_debian_sysroot//:eglstream",
        "@arm64_debian_sysroot//:nvbufsurface",
    ],
)

cc_binary(
    name = "gpu_apriltag",
    srcs = ["gpu_apriltag.cc"],
    features = ["cuda"],
    deps = [
        ":apriltag",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:charuco_lib",
        "//third_party:opencv",
    ],
)
