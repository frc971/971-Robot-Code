load("//frc971:halide.bzl", "halide_library")

halide_library(
    name = "ycbcr",
    src = "crcv_generator.cc",
    args = "rows=1088 cols=1456 ystride=2048 cbcrstride=3840",
    function = "ycbcr",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cuda",
    srcs = [
        "apriltag.cc",
        "cuda.cc",
        "labeling_allegretti_2019_BKE.cc",
        "points.cc",
        "threshold.cc",
    ],
    hdrs = [
        "apriltag.h",
        "cuda.h",
        "gpu_image.h",
        "labeling_allegretti_2019_BKE.h",
        "points.h",
        "threshold.h",
    ],
    copts = [
        "-Wno-pass-failed",
        #"-DCUB_DETAIL_DEBUG_ENABLE_LOG=1",
        #"-DDEBUG=1",
    ],
    features = ["cuda"],
    deps = [
        "//aos/time",
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
    ],
    features = ["cuda"],
    deps = [
        ":cuda",
        ":ycbcr",
        "//aos:flatbuffer_merge",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//frc971/vision:vision_fbs",
        "//third_party:cudart",
        "//third_party:opencv",
        "//third_party/apriltag",
        "//y2023/vision:ThresholdHalide",
        "//y2023/vision:ToGreyscaleAndDecimateHalide",
    ],
)
