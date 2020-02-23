load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library")

py_binary(
    name = "load_sift_training",
    data = [
        ":test_images/train_power_port_red.png",
        ":test_images/train_power_port_red_webcam.png",
        ":test_images/train_power_port_blue.png",
        ":test_images/train_loading_bay_red.png",
        ":test_images/train_loading_bay_blue.png",
    ],
    srcs = ["load_sift_training.py",
        "camera_definition.py",
        "define_training_data.py",
        "target_definition.py",
        "train_and_match.py",
    ],
    args = ["sift_training_data.h",
    ],
    default_python_version = "PY3",
    srcs_version = "PY2AND3",
    deps = [
        "//y2020/vision/sift:sift_fbs_python",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
        "@bazel_tools//tools/python/runfiles",
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
    name = "sift_training",
    hdrs = [
        "sift_training_data.h",
    ],
    visibility = ["//visibility:public"],
)
