load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library")
load("//aos:config.bzl", "aos_config")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = ["vision.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "vision_ts_fbs",
    srcs = ["vision.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "calibration_fbs",
    srcs = ["calibration.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "calibration_ts_fbs",
    srcs = ["calibration.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "target_map_ts_fbs",
    srcs = ["target_map.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
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
    name = "create_calib_file",
    srcs = [
        "create_calib_file.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@pip//glog",
        "@pip//opencv_python",
    ],
)

cc_library(
    name = "v4l2_reader",
    srcs = [
        "v4l2_reader.cc",
    ],
    hdrs = [
        "v4l2_reader.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos/events:epoll",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "//aos/util:threaded_consumer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)

cc_library(
    name = "charuco_lib",
    srcs = [
        "charuco_lib.cc",
    ],
    hdrs = [
        "charuco_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:team_number",
        "//frc971/control_loops:quaternion_utils",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "extrinsics_calibration",
    srcs = [
        "calibration_accumulator.cc",
        "calibration_accumulator.h",
        "extrinsics_calibration.cc",
        "extrinsics_calibration.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":charuco_lib",
        ":foxglove_image_converter_lib",
        "//aos:init",
        "//aos/events/logging:log_reader",
        "//frc971/analysis:in_process_plotter",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/vision:visualize_robot",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:CompressedImage_schema",
        "@com_github_foxglove_schemas//:ImageAnnotations_schema",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

flatbuffer_cc_library(
    name = "target_map_fbs",
    srcs = ["target_map.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "target_mapper",
    srcs = ["target_mapper.cc"],
    hdrs = ["target_mapper.h"],
    data = ["target_map.json"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":geometry_lib",
        ":target_map_fbs",
        "//aos/events:simulated_event_loop",
        "//frc971/control_loops:control_loop",
        "//frc971/vision:visualize_robot",
        "//frc971/vision/ceres:pose_graph_3d_lib",
        "//third_party:opencv",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "target_mapper_test",
    srcs = [
        "target_mapper_test.cc",
    ],
    data = [":target_map.json"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":target_mapper",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//aos/util:math",
    ],
)

cc_library(
    name = "geometry_lib",
    hdrs = [
        "geometry.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/util:math",
        "//third_party:opencv",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "geometry_test",
    srcs = [
        "geometry_test.cc",
    ],
    deps = [
        ":geometry_lib",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "visualize_robot",
    srcs = [
        "visualize_robot.cc",
    ],
    hdrs = [
        "visualize_robot.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//third_party:opencv",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "visualize_robot_sample",
    srcs = [
        "visualize_robot_sample.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//frc971/vision:visualize_robot",
        "//third_party:opencv",
        "@com_github_google_glog//:glog",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "media_device",
    srcs = [
        "media_device.cc",
    ],
    hdrs = ["media_device.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/scoped:scoped_fd",
        "//aos/util:file",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "foxglove_image_converter_lib",
    srcs = ["foxglove_image_converter_lib.cc"],
    hdrs = ["foxglove_image_converter_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":charuco_lib",
        ":vision_fbs",
        "//aos/events:event_loop",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
    ],
)

aos_config(
    name = "converter_config",
    testonly = True,
    src = "converter_test_config.json",
    flatbuffers = [
        "//frc971/vision:vision_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
)

cc_test(
    name = "foxglove_image_converter_test",
    srcs = ["foxglove_image_converter_test.cc"],
    data = [
        ":converter_config",
        "@april_tag_test_image",
    ],
    deps = [
        ":foxglove_image_converter_lib",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_library(
    name = "intrinsics_calibration_lib",
    srcs = [
        "intrinsics_calibration_lib.cc",
    ],
    hdrs = [
        "intrinsics_calibration_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:event_loop",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:vision_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "intrinsics_calibration",
    srcs = [
        "intrinsics_calibration.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = [
        "//y2020:__subpackages__",
        "//y2022:__subpackages__",
        "//y2023:__subpackages__",
    ],
    deps = [
        ":intrinsics_calibration_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:vision_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
