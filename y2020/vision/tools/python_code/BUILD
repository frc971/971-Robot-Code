load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library")
load("//tools:platforms.bzl", "platforms")

py_binary(
    name = "load_sift_training",
    srcs = [
        "camera_definition.py",
        "define_training_data.py",
        "load_sift_training.py",
        "target_definition.py",
        "train_and_match.py",
    ],
    args = [
        "sift_training_data.h",
    ],
    data = glob(["calib_files/*.json"]) + glob([
        "test_images/*.png",
    ]),
    python_version = "PY3",
    srcs_version = "PY2AND3",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
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
