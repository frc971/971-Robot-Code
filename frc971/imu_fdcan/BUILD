load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "dual_imu_fbs",
    srcs = ["dual_imu.fbs"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "can_translator_status_fbs",
    srcs = ["can_translator_status.fbs"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "can_translator",
    srcs = ["can_translator_main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":can_translator_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "can_translator_lib",
    srcs = [
        "can_translator_lib.cc",
    ],
    hdrs = [
        "can_translator_lib.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":can_translator_status_fbs",
        ":dual_imu_fbs",
        "//aos/events:event_loop",
        "//frc971/can_logger:can_logging_fbs",
    ],
)

cc_test(
    name = "can_translator_lib_test",
    srcs = [
        "can_translator_lib_test.cc",
    ],
    data = [
        ":can_translator_test_config",
    ],
    deps = [
        ":can_translator_lib",
        ":can_translator_status_fbs",
        ":dual_imu_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//frc971/can_logger:can_logging_fbs",
        "@com_github_google_glog//:glog",
    ],
)

aos_config(
    name = "can_translator_test_config",
    src = "can_translator_test_config_source.json",
    flatbuffers = [
        "//aos/logging:log_message_fbs",
        ":dual_imu_fbs",
        ":can_translator_status_fbs",
        "//frc971/can_logger:can_logging_fbs",
        "//aos/events:event_loop_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
