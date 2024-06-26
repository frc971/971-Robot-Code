load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:js.bzl", "ts_project")

ts_project(
    name = "vision_plotter",
    srcs = ["vision_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":target_estimate_ts_fbs",
        "//aos:configuration_ts_fbs",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//aos/network/www:reflection_ts",
        "@com_github_google_flatbuffers//reflection:reflection_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

py_library(
    name = "camera_definition",
    srcs = [
        "camera_definition.py",
    ],
    deps = [
        "@pip//glog",
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
        "//frc971/vision:calibration_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@pip//glog",
        "@pip//opencv_python",
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
        ":target_estimate_fbs",
        ":target_estimator_lib",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/vision:calibration_fbs",
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

cc_binary(
    name = "ball_color_detector",
    srcs = [
        "ball_color_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":ball_color_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_test(
    name = "ball_color_test",
    srcs = [
        "ball_color_test.cc",
    ],
    data = [
        "test_ball_color_image.jpg",
    ],
    deps = [
        ":ball_color_lib",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//y2022:constants",
    ],
)

cc_library(
    name = "ball_color_lib",
    srcs = [
        "ball_color.cc",
    ],
    hdrs = [
        "ball_color.h",
    ],
    data = [
        "//y2022:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":ball_color_fbs",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//frc971/input:joystick_state_fbs",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2022:constants",
    ],
)

static_flatbuffer(
    name = "ball_color_fbs",
    srcs = ["ball_color.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//frc971/input:joystick_state_fbs",
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
        "//frc971/vision:geometry_lib",
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
        ":blob_detector_lib",
        ":target_estimate_fbs",
        "//aos/logging",
        "//aos/time",
        "//frc971/control_loops:quaternion_utils",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:geometry_lib",
        "//third_party:opencv",
        "//y2022:constants",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

static_flatbuffer(
    name = "target_estimate_fbs",
    srcs = ["target_estimate.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//frc971/vision:calibration_fbs",
    ],
)

flatbuffer_ts_library(
    name = "target_estimate_ts_fbs",
    srcs = ["target_estimate.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/vision:calibration_ts_fbs",
    ],
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
        ":calibration_data",
        ":camera_reader_lib",
        ":target_estimator_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "viewer_replay",
    srcs = [
        "viewer_replay.cc",
    ],
    data = [
        "//y2022:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":blob_detector_lib",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/input:joystick_state_fbs",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2022/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_binary(
    name = "calibrate_extrinsics",
    srcs = [
        "calibrate_extrinsics.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events/logging:log_reader",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/vision:extrinsics_calibration",
        "//third_party:opencv",
        "//y2022/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_binary(
    name = "image_decimator",
    srcs = ["image_decimator.cc"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//aos:flatbuffers",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:vision_fbs",
    ],
)

filegroup(
    name = "image_streamer_start",
    srcs = ["image_streamer_start.sh"],
    visibility = ["//visibility:public"],
)
