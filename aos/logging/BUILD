load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")

# The primary client logging interface.
cc_library(
    name = "logging",
    srcs = [
        "context.cc",
        "implementations.cc",
        "interface.cc",
        "printf_formats.h",
    ],
    hdrs = [
        "context.h",
        "implementations.h",
        "interface.h",
        "logging.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:die",
        "//aos:macros",
        "//aos/libc:aos_strerror",
        "//aos/time",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "log_namer",
    srcs = ["log_namer.cc"],
    hdrs = ["log_namer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "implementations_test",
    srcs = [
        "implementations_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logging",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

flatbuffer_cc_library(
    name = "log_message_fbs",
    srcs = ["log_message.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_static_flatbuffer(
    name = "log_message_schema",
    function = "aos::logging::LogMessageFbsSchema",
    target = ":log_message_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "dynamic_log_command_fbs",
    srcs = [
        "dynamic_log_command.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "dynamic_logging",
    srcs = ["dynamic_logging.cc"],
    hdrs = ["dynamic_logging.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamic_log_command_fbs",
        "//aos/events:shm_event_loop",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "dynamic_logging_test",
    srcs = ["dynamic_logging_test.cc"],
    data = [
        "//aos/events:pingpong_config",
    ],
    deps = [
        ":dynamic_logging",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)
