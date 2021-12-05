load(":fast_gaussian.bzl", "fast_gaussian")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library", "flatbuffer_ts_library")

cc_binary(
    name = "fast_gaussian_generator",
    srcs = [
        "fast_gaussian_generator.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//third_party:halide",
        "//third_party:halide_gengen",
        "//third_party:opencv",
        "@com_github_google_glog//:glog",
    ],
)

py_binary(
    name = "fast_gaussian_runner",
    srcs = [
        "fast_gaussian_runner.py",
    ],
    data = [
        ":fast_gaussian_generator",
        "@amd64_debian_sysroot//:sysroot_files",
    ],
    main = "fast_gaussian_runner.py",
    target_compatible_with = ["@platforms//os:linux"],
    toolchains = [
        "@bazel_tools//tools/cpp:current_cc_toolchain",
    ],
    deps = [
        "@bazel_tools//tools/python/runfiles",
    ],
)

# Each element is [sigma, sigma_name, radius].
# opencv's default width is calculated as:
#   cvRound(sigma1 * (depth == CV_8U ? 3 : 4) * 2 + 1) | 1
# Pulling that in helps a lot with making it faster (less data to read, and less
# math to do), but if you make it too narrow SIFT quickly freaks out.
sigmas = [
    [
        "1.2262734984654078",
        "1p2",
        "9",
    ],
    [
        "1.5450077936447955",
        "1p5",
        "11",
    ],
    [
        "1.9465878414647133",
        "1p9",
        "13",
    ],
    [
        "2.4525469969308156",
        "2p4",
        "15",
    ],
    [
        "3.0900155872895909",
        "3p1",
        "19",
    ],
    # TODO(Brian): We only need one of these two for 1280x720. Don't generate
    # all the redundant versions for other sizes, and maybe stop doing the one
    # we don't actually use.
    [
        "1.2489995956420898",
        "1p24",
        "11",
    ],
    [
        "1.5198683738708496",
        "1p52",
        "15",
    ],
]

sizes = [
    [
        1280,
        960,
    ],
    [
        640,
        480,
    ],
    [
        320,
        240,
    ],
    [
        160,
        120,
    ],
    [
        80,
        60,
    ],
]

fast_gaussian(sigmas, sizes)

cc_library(
    name = "sift971",
    srcs = [
        "sift971.cc",
    ],
    hdrs = [
        "sift971.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":fast_gaussian",
        "//third_party:opencv",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "fast_gaussian",
    srcs = [
        "fast_gaussian.cc",
    ],
    hdrs = [
        "fast_gaussian.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":fast_gaussian_all",
        "//third_party:halide_runtime",
        "//third_party:opencv",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "fast_gaussian_test",
    srcs = [
        "fast_gaussian_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":fast_gaussian",
        "//aos/testing:googletest",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "testing_sift",
    srcs = [
        "testing_sift.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":fast_gaussian",
        "//aos:init",
        "//aos/time",
        "//third_party:opencv",
        "//y2020/vision/sift:sift971",
        "@com_github_google_glog//:glog",
    ],
)

flatbuffer_py_library(
    name = "sift_fbs_python",
    srcs = [
        "sift.fbs",
        "sift_training.fbs",
    ],
    namespace = "frc971.vision.sift",
    tables = [
        "KeypointFieldLocation",
        "Feature",
        "Match",
        "ImageMatch",
        "TransformationMatrix",
        "CameraCalibration",
        "CameraPose",
        "ImageMatchResult",
        "TrainingImage",
        "TrainingData",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "sift_fbs",
    srcs = ["sift.fbs"],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "sift_ts_fbs",
    srcs = ["sift.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
)

flatbuffer_cc_library(
    name = "sift_training_fbs",
    srcs = ["sift_training.fbs"],
    gen_reflections = True,
    includes = [":sift_fbs_includes"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "demo_sift_training",
    srcs = ["demo_sift_training.py"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sift_fbs_python",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

genrule(
    name = "run_demo_sift_training",
    srcs = [
        "images/demo/FRC-Image4-cleaned.png",
    ],
    outs = [
        "demo_sift.h",
    ],
    cmd = " ".join([
        "$(location :demo_sift_training)",
        "$(location images/demo/FRC-Image4-cleaned.png)",
        "$(location demo_sift.h)",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":demo_sift_training",
    ],
)

cc_library(
    name = "demo_sift",
    hdrs = [
        "demo_sift.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
