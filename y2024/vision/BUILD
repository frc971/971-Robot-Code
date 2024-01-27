filegroup(
    name = "image_streamer_start",
    srcs = ["image_streamer_start.sh"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "foxglove_image_converter",
    srcs = ["foxglove_image_converter.cc"],
    visibility = ["//y2024:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:foxglove_image_converter_lib",
    ],
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
        "//y2024/constants:constants_fbs",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_nvidia_cccl//:cccl",
        "@com_github_nvidia_cuco//:cuco",
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
        "//y2024:__subpackages__",
    ],
    deps = [
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:vision_fbs",
        "//frc971/vision:vision_util_lib",
        "//third_party:opencv",
        "//y2024/constants:constants_fbs",
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
        "//y2024:aos_config",
        "//y2024/constants:constants.json",
        "//y2024/vision:maps",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2024:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:pose",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:extrinsics_calibration",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_util_lib",
        "//third_party:opencv",
        "//y2024/constants:constants_fbs",
        "//y2024/constants:simulated_constants_sender",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
