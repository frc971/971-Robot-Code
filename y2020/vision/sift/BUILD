load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_py_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load(":fast_gaussian.bzl", "fast_gaussian")

# Note that this file is also used directly by :fast_gaussian_halide_generator,
# without any dependencies added here.
cc_library(
    name = "get_gaussian_kernel",
    hdrs = [
        "get_gaussian_kernel.h",
    ],
)

cc_test(
    name = "get_gaussian_kernel_test",
    srcs = [
        "get_gaussian_kernel_test.cc",
    ],
    deps = [
        ":get_gaussian_kernel",
        "//aos/testing:googletest",
        "//third_party:opencv",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

sh_binary(
    name = "fast_gaussian_halide_generator",
    srcs = [
        "fast_gaussian_halide_generator.sh",
    ],
    data = [
        "fast_gaussian_generator.cc",
        "get_gaussian_kernel.h",
        "@amd64_debian_sysroot//:sysroot_files",
        "@clang_amd64_deps//:all",
        "@deb_zlib1g_dev_1_2_11_dfsg_2_amd64_deb_repo//file",
        "@halide_k8//:build_files",
        "@llvm_toolchain//:all-components-x86_64-linux",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

genrule(
    name = "run_fast_gaussian_halide_generator",
    outs = [
        "fast_gaussian_generator",
    ],
    cmd = "$(location :fast_gaussian_halide_generator) $@",
    tools = [
        ":fast_gaussian_halide_generator",
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
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
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
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
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
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
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

static_flatbuffer(
    name = "sift_fbs",
    srcs = ["sift.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "sift_ts_fbs",
    srcs = ["sift.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
)

static_flatbuffer(
    name = "sift_training_fbs",
    srcs = ["sift_training.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [":sift_fbs"],
)

py_binary(
    name = "demo_sift_training",
    srcs = ["demo_sift_training.py"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sift_fbs_python",
        "@pip//opencv_python",
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
