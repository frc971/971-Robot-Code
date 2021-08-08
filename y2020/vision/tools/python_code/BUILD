load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library")
load("//tools:platforms.bzl", "platforms")

py_library(
    name = "train_and_match",
    srcs = ["train_and_match.py"],
    data = [
        "@python_repo//:scipy",
    ],
    deps = [
        "//external:python-glog",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

py_library(
    name = "define_training_data",
    srcs = [
        "define_training_data.py",
    ],
    data = [
        "@python_repo//:scipy",
    ],
    deps = [
        ":train_and_match",
        "//external:python-glog",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

py_library(
    name = "camera_definition",
    srcs = [
        "camera_definition.py",
    ],
    deps = [
        ":define_training_data",
        "//external:python-glog",
    ],
)

py_library(
    name = "target_definition",
    srcs = [
        "target_definition.py",
    ],
    deps = [
        ":camera_definition",
        ":define_training_data",
        ":train_and_match",
        "//external:python-glog",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

py_binary(
    name = "target_definition_main",
    srcs = ["target_definition.py"],
    data = glob(["calib_files/*.json"]) + glob([
        "test_images/*.png",
    ]),
    main = "target_definition.py",
    python_version = "PY3",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":target_definition",
        "@bazel_tools//tools/python/runfiles",
    ],
)

py_binary(
    name = "load_sift_training",
    srcs = [
        "load_sift_training.py",
    ],
    args = [
        "sift_training_data.h",
    ],
    data = glob(["calib_files/*.json"]) + glob([
        "test_images/*.png",
    ]),
    python_version = "PY3",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_definition",
        ":target_definition",
        "//external:python-glog",
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

py_binary(
    name = "image_match_test",
    srcs = [
        "image_match_test.py",
    ],
    data = glob(["calib_files/*.json"]) + glob([
        "test_images/*.png",
    ]),
    python_version = "PY3",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_definition",
        ":target_definition",
        "//external:python-glog",
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

genrule(
    name = "run_load_sift_training",
    outs = [
        "sift_training_data.h",
    ],
    cmd = " ".join([
        "$(location :load_sift_training)",
        "$(location sift_training_data.h)",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":load_sift_training",
    ],
)

cc_library(
    name = "sift_training_data",
    hdrs = [
        "sift_training_data.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

py_binary(
    name = "load_sift_training_test",
    srcs = [
        "camera_definition_test.py",
        "define_training_data.py",
        "load_sift_training.py",
        "target_definition_test.py",
        "train_and_match.py",
    ],
    args = [
        "sift_training_data_test.h",
        "test",
    ],
    data = glob(["calib_files/*.json"]) + glob([
        "test_images/*.png",
    ]),
    main = "load_sift_training.py",
    python_version = "PY3",
    srcs_version = "PY2AND3",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":load_sift_training",
        "//external:python-glog",
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

genrule(
    name = "run_load_sift_training_test",
    outs = [
        "sift_training_data_test.h",
    ],
    cmd = " ".join([
        "$(location :load_sift_training_test)",
        "$(location sift_training_data_test.h) test",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":load_sift_training_test",
    ],
)

cc_library(
    name = "sift_training_data_test",
    hdrs = [
        "sift_training_data_test.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "camera_param_test",
    srcs = [
        "camera_param_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sift_training_data_test",
        "//aos/testing:googletest",
        "//third_party:opencv",
        "//y2020/vision:vision_fbs",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
    ],
)
