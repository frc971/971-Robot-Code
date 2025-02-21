filegroup(
    name = "image_streamer_start",
    srcs = ["image_streamer_start.sh"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "apriltag_detector",
    srcs = [
        "apriltag_detector.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    features = ["cuda"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/orin:gpu_apriltag_lib",
        "//third_party:cudart",
        "//third_party/apriltag",
        "//y2025/constants:constants_fbs",
        "@com_github_nvidia_cccl//:cccl",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = [
        "//y2025:__subpackages__",
    ],
    deps = [
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:vision_fbs",
        "//frc971/vision:vision_util_lib",
        "//third_party:opencv",
        "//y2025/constants:constants_fbs",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "calibrate_multi_cameras",
    srcs = [
        "calibrate_multi_cameras.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    data = [
        "//y2025:aos_config",
        "//y2025/constants:constants.json",
        "//y2025/vision:maps",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2025:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:pose",
        "//frc971/vision:calibrate_multi_cameras_lib",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:extrinsics_calibration",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_util_lib",
        "//third_party:opencv",
        "//y2025/constants:constants_fbs",
        "//y2025/constants:simulated_constants_sender",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "frame_logger_lib",
    srcs = ["frame_logger.cc"],
    hdrs = ["frame_logger.h"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    deps = [
        "//aos/events:event_loop",
        "//aos/time",
        "//frc971/vision:vision_fbs",
        "@arm64_yocto_sysroot//:opencv",
        "@com_google_absl//absl/log",
    ],
)

cc_binary(
    name = "frame_logger",
    srcs = ["frame_logger_main.cc"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_logger_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)
