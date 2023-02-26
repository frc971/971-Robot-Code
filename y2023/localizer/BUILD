load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

flatbuffer_cc_library(
    name = "status_fbs",
    srcs = [
        "status.fbs",
    ],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/imu_reader:imu_failures_fbs",
    ],
)

flatbuffer_ts_library(
    name = "status_ts_fbs",
    srcs = ["status.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:drivetrain_status_ts_fbs",
        "//frc971/imu_reader:imu_failures_ts_fbs",
    ],
)

flatbuffer_cc_library(
    name = "visualization_fbs",
    srcs = [
        "visualization.fbs",
    ],
    gen_reflections = True,
    visibility = ["//visibility:public"],
    deps = [
        ":status_fbs",
    ],
)

flatbuffer_ts_library(
    name = "visualization_ts_fbs",
    srcs = ["visualization.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":status_ts_fbs",
    ],
)

flatbuffer_cc_library(
    name = "scoring_map_fbs",
    srcs = [
        "scoring_map.fbs",
    ],
    gen_reflections = True,
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/vision:target_map_fbs",
    ],
)

flatbuffer_cc_library(
    name = "relative_scoring_map_fbs",
    srcs = [
        "relative_scoring_map.fbs",
    ],
    gen_reflections = True,
    visibility = ["//visibility:public"],
    deps = [
        ":scoring_map_fbs",
    ],
)

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
    deps = [
        "//frc971/vision:target_map_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "map_expander_lib",
    srcs = ["map_expander_lib.cc"],
    hdrs = ["map_expander_lib.h"],
    deps = [
        ":relative_scoring_map_fbs",
        ":scoring_map_fbs",
        ":utils",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
    ],
)

cc_binary(
    name = "map_expander",
    srcs = ["map_expander.cc"],
    data = [
        "//y2023/constants:scoring_map",
        "//y2023/vision/maps",
    ],
    deps = [
        ":map_expander_lib",
        "//aos:init",
    ],
)

cc_test(
    name = "map_expander_lib_test",
    srcs = ["map_expander_lib_test.cc"],
    data = [
        "//y2023/constants:scoring_map",
        "//y2023/vision/maps",
    ],
    deps = [
        ":map_expander_lib",
        "//aos/testing:flatbuffer_eq",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "localizer",
    srcs = ["localizer.cc"],
    hdrs = ["localizer.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":status_fbs",
        ":utils",
        ":visualization_fbs",
        "//aos/containers:sized_array",
        "//aos/events:event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:pose",
        "//frc971/control_loops/drivetrain:hybrid_ekf",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "//frc971/control_loops/drivetrain:localizer_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/control_loops/drivetrain/localization:utils",
        "//frc971/imu_reader:imu_watcher",
        "//frc971/vision:target_map_fbs",
        "//y2023:constants",
        "//y2023/constants:constants_fbs",
    ],
)

cc_test(
    name = "localizer_test",
    srcs = ["localizer_test.cc"],
    data = ["//y2023:aos_config"],
    deps = [
        ":localizer",
        ":status_fbs",
        ":utils",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//frc971/control_loops/drivetrain:drivetrain_test_lib",
        "//frc971/control_loops/drivetrain:localizer_fbs",
        "//y2023/constants:simulated_constants_sender",
        "//y2023/control_loops/drivetrain:drivetrain_base",
    ],
)

cc_binary(
    name = "localizer_main",
    srcs = ["localizer_main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":localizer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//y2023/control_loops/drivetrain:drivetrain_base",
    ],
)
