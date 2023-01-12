py_library(
    name = "train_and_match",
    srcs = ["train_and_match.py"],
    deps = [
        "@pip//glog",
        "@pip//opencv_python",
        "@pip//scipy",
    ],
)

py_library(
    name = "define_training_data",
    srcs = [
        "define_training_data.py",
    ],
    deps = [
        ":train_and_match",
        "@pip//glog",
        "@pip//opencv_python",
        "@pip//scipy",
    ],
)

py_library(
    name = "camera_definition",
    srcs = [
        "camera_definition.py",
    ],
    deps = [
        ":define_training_data",
        "@pip//glog",
    ],
)

py_library(
    name = "target_definition",
    srcs = [
        "target_definition.py",
    ],
    data = glob(["target_definitions/*.json"]),
    deps = [
        ":camera_definition",
        ":define_training_data",
        ":train_and_match",
        "@pip//glog",
        "@pip//opencv_python",
    ],
)

py_binary(
    name = "target_definition_main",
    srcs = ["target_definition.py"],
    data = glob(["calib_files/*.json"]) + glob([
        "test_images/*.png",
    ]),
    main = "target_definition.py",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_definition",
        ":target_definition",
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@pip//glog",
        "@pip//opencv_python",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_definition",
        ":target_definition",
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@pip//glog",
        "@pip//opencv_python",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":load_sift_training",
        "//y2020/vision/sift:sift_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@pip//glog",
        "@pip//opencv_python",
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
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
    ],
)
