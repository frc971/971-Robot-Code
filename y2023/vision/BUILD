load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

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
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/vision:create_calib_file",
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
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
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
        ":calibration_data",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//frc971/control_loops:pose",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_mapper",
        "//third_party:opencv",
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
        ":calibration_data",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:charuco_lib",
        "//frc971/vision:target_map_fbs",
        "//frc971/vision:target_mapper",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//third_party/apriltag",
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
