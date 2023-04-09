load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//tools/build_rules:select.bzl", "cpu_select")

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader.cc",
        "rkisp1-config.h",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "//tools/platforms/hardware:raspberry_pi",
    ],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:media_device",
        "//frc971/vision:v4l2_reader",
    ],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2023/vision:vision_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "localization_verifier",
    srcs = [
        "localization_verifier.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/vision:vision_fbs",
        "//y2023/localizer",
        "//y2023/localizer:utils",
        "//y2023/vision:vision_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "target_mapping",
    srcs = [
        "target_mapping.cc",
    ],
    data = [
        "//y2023:aos_config",
        "//y2023/constants:constants.json",
        "//y2023/vision:maps",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        ":aprilrobotics_lib",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:pose",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_mapper",
        "//third_party:opencv",
        "//y2023/constants:constants_fbs",
        "//y2023/constants:simulated_constants_sender",
    ],
)

cc_library(
    name = "vision_util",
    srcs = ["vision_util.cc"],
    hdrs = ["vision_util.h"],
    deps = [
        "//third_party:opencv",
        "//y2023/constants:constants_fbs",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "aprilrobotics_lib",
    srcs = [
        "aprilrobotics.cc",
        "aprilrobotics.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        ":vision_util",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_map_fbs",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_fbs",
        "//frc971/vision:visualize_robot",
        "//third_party:opencv",
        "//third_party/apriltag",
        "//y2023/constants:constants_fbs",
    ],
)

cc_binary(
    name = "aprilrobotics",
    srcs = [
        "aprilrobotics_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        ":aprilrobotics_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_test(
    name = "april_detection_test",
    srcs = [
        "april_detection_test.cc",
    ],
    data = [
        "//y2023:aos_config",
        "//y2023/constants:constants.json",
        "@apriltag_test_bfbs_images",
    ],
    deps = [
        ":aprilrobotics_lib",
        "//aos:flatbuffer_merge",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:test_logging",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_fbs",
        "//y2023/constants:constants_fbs",
        "//y2023/constants:constants_list_fbs",
    ],
)

filegroup(
    name = "image_streamer_start",
    srcs = ["image_streamer_start.sh"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "foxglove_image_converter",
    srcs = ["foxglove_image_converter.cc"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:foxglove_image_converter_lib",
    ],
)

cc_binary(
    name = "calibrate_extrinsics",
    srcs = [
        "calibrate_extrinsics.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events/logging:log_reader",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/vision:extrinsics_calibration",
        "//third_party:opencv",
        "//y2023/constants:constants_fbs",
        "//y2023/vision:vision_util",
    ],
)

cc_binary(
    name = "game_pieces_detector",
    srcs = [
        "game_pieces_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        ":game_pieces_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "game_pieces_lib",
    srcs = [
        "game_pieces.cc",
    ],
    hdrs = [
        "game_pieces.h",
    ],
    data = [
        "//y2023:aos_config",
        "@game_pieces_edgetpu_model//file",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        ":game_pieces_fbs",
        ":yolov5_lib",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//frc971/vision:vision_fbs",
    ],
)

flatbuffer_cc_library(
    name = "game_pieces_fbs",
    srcs = ["game_pieces.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "game_pieces_ts_fbs",
    srcs = ["game_pieces.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "image_logger",
    srcs = [
        "image_logger.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/logging:log_namer",
        "//frc971/input:joystick_state_fbs",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "yolov5_lib",
    srcs = ["yolov5.cc"],
    hdrs = ["yolov5.h"],
    copts = ["-Wno-unused-parameter"],
    deps = [
        "//third_party:opencv",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ] + cpu_select({
        "amd64": [
            "@libtensorflowlite//:tensorflow-k8",
            "@libedgetpu//:libedgetpu-k8",
        ],
        "arm": [
            "@libtensorflowlite//:tensorflow-arm",
            "@libedgetpu//:libedgetpu-arm",
        ],
    }),
)

filegroup(
    name = "game_pieces_detector_starter",
    srcs = ["game_pieces_detector_starter.sh"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "camera_monitor_lib",
    srcs = ["camera_monitor_lib.cc"],
    hdrs = ["camera_monitor_lib.h"],
    deps = [
        "//aos/events:event_loop",
        "//aos/starter:starter_rpc_lib",
        "//frc971/vision:vision_fbs",
    ],
)

cc_binary(
    name = "camera_monitor",
    srcs = ["camera_monitor.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_monitor_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
