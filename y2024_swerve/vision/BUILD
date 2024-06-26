filegroup(
    name = "image_streamer_start",
    srcs = ["image_streamer_start.sh"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "target_mapping",
    srcs = [
        "target_mapping.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    data = [
        "//y2024_swerve:aos_config",
        "//y2024_swerve/constants:constants.json",
        "//y2024_swerve/vision:maps",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2024_swerve:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:pose",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_util_lib",
        "//frc971/vision:visualize_robot",
        "//third_party:opencv",
        "//y2024_swerve/constants:constants_fbs",
        "//y2024_swerve/constants:simulated_constants_sender",
    ],
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
        "//y2024_swerve/constants:constants_fbs",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_nvidia_cccl//:cccl",
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
        "//y2024_swerve:__subpackages__",
    ],
    deps = [
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:vision_fbs",
        "//frc971/vision:vision_util_lib",
        "//third_party:opencv",
        "//y2024_swerve/constants:constants_fbs",
        "@com_google_absl//absl/strings",
    ],
)
