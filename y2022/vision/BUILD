load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library")

flatbuffer_cc_library(
    name = "calibration_fbs",
    srcs = ["calibration.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
)

flatbuffer_py_library(
    name = "calibration_fbs_python",
    srcs = [
        "calibration.fbs",
    ],
    namespace = "frc971.vision.calibration",
    tables = [
        "CalibrationData",
        "CameraCalibration",
        "TransformationMatrix",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "camera_definition",
    srcs = [
        "camera_definition.py",
    ],
    deps = [
        "//external:python-glog",
    ],
)

py_binary(
    name = "create_calib_file",
    srcs = [
        "create_calib_file.py",
    ],
    args = [
        "calibration_data.h",
    ],
    data = glob(["calib_files/*.json"]),
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_definition",
        "//external:python-glog",
        "//y2022/vision:calibration_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@opencv_contrib_nonfree_amd64//:python_opencv",
    ],
)

genrule(
    name = "run_calibration_data",
    outs = [
        "calibration_data.h",
    ],
    cmd = " ".join([
        "$(location :create_calib_file)",
        "$(location calibration_data.h)",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":create_calib_file",
    ],
)

cc_library(
    name = "calibration_data",
    hdrs = [
        "calibration_data.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "camera_reader_lib",
    srcs = [
        "camera_reader.cc",
    ],
    hdrs = [
        "camera_reader.h",
        "gpio.h",
    ],
    data = [
        "//y2022:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":blob_detector_lib",
        ":calibration_data",
        ":calibration_fbs",
        ":target_estimate_fbs",
        ":target_estimator_lib",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "//frc971/vision:v4l2_reader",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":camera_reader_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "blob_detector_lib",
    srcs = [
        "blob_detector.cc",
    ],
    hdrs = [
        "blob_detector.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//aos/network:team_number",
        "//aos/time",
        "//third_party:opencv",
    ],
)

cc_library(
    name = "target_estimator_lib",
    srcs = [
        "target_estimator.cc",
    ],
    hdrs = [
        "target_estimator.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":target_estimate_fbs",
        "//third_party:opencv",
    ],
)

flatbuffer_cc_library(
    name = "target_estimate_fbs",
    srcs = ["target_estimate.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
    ],
    data = [
        "//y2022:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":blob_detector_lib",
        ":target_estimator_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
    ],
)
