load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library")

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
    data = [
        ":test_images/train_loading_bay_blue.png",
        ":test_images/train_loading_bay_red.png",
        ":test_images/train_power_port_blue.png",
        ":test_images/train_power_port_red.png",
        ":test_images/train_power_port_red_webcam.png",
    ],
    default_python_version = "PY3",
    srcs_version = "PY2AND3",
    deps = [
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
        "//external:python-glog",
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
    tools = [
        ":load_sift_training",
    ],
)

cc_library(
    name = "sift_training_data",
    hdrs = [
        "sift_training_data.h",
    ],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "load_sift_training_test",
    srcs = [
        "camera_definition.py",
        "define_training_data.py",
        "load_sift_training.py",
        "target_definition.py",
        "train_and_match.py",
    ],
    args = [
        "sift_training_data_test.h",
        "test",
    ],
    data = [
        ":test_images/train_power_port_red.png",
    ],
    default_python_version = "PY3",
    main = "load_sift_training.py",
    srcs_version = "PY2AND3",
    deps = [
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
        "//external:python-glog",
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
    tools = [
        ":load_sift_training_test",
    ],
)

cc_library(
    name = "sift_training_data_test",
    hdrs = [
        "sift_training_data_test.h",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "camera_param_test",
    srcs = [
        "camera_param_test.cc",
    ],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    deps = [
        ":sift_training_data_test",
        "//aos/testing:googletest",
        "//third_party:opencv",
        "//y2020/vision:vision_fbs",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
    ],
)
