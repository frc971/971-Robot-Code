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
