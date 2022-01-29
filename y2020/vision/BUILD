load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "galactic_search_path_fbs",
    srcs = ["galactic_search_path.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
)

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        ":camera_reader_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "camera_reader_lib",
    srcs = [
        "camera_reader.cc",
    ],
    hdrs = [
        "camera_reader.h",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"] + ["//y2022:__subpackages__"],
    deps = [
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//aos/network:team_number",
        "//frc971/vision:v4l2_reader",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2020/vision/sift:sift971",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision/tools/python_code:sift_training_data",
    ],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:v4l2_reader",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2020/vision/sift:sift_fbs",
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
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:team_number",
        "//frc971/control_loops:quaternion_utils",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision/tools/python_code:sift_training_data",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "calibration",
    srcs = [
        "calibration.cc",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        ":charuco_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/vision:vision_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//third_party:opencv",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision/tools/python_code:sift_training_data",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "viewer_replay",
    srcs = [
        "viewer_replay.cc",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "extrinsics_calibration",
    srcs = [
        "calibration_accumulator.cc",
        "calibration_accumulator.h",
        "extrinsics_calibration.cc",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        ":charuco_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_reader",
        "//frc971/analysis:in_process_plotter",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
