load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

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
        "//y2023/vision:april_debug_fbs",
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
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        ":aprilrobotics_lib",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:pose",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_mapper",
        "//third_party:opencv",
        "//y2023/constants:constants_fbs",
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

flatbuffer_cc_library(
    name = "april_debug_fbs",
    srcs = ["april_debug.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
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
        ":april_debug_fbs",
        ":vision_util",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_map_fbs",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_fbs",
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
