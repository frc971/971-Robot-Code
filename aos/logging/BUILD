load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")

# The primary client logging interface.
cc_library(
    name = "logging",
    srcs = [
        "context.cc",
        "implementations.cc",
        "interface.cc",
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
        ":printf_formats",
        ":sizes",
        "//aos:die",
        "//aos:macros",
        "//aos:thread_local",
        "//aos/libc:aos_strerror",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/type_traits",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)

cc_library(
    name = "log_namer",
    srcs = ["log_namer.cc"],
    hdrs = ["log_namer.h"],
    copts = ["-Wno-format-nonliteral"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "sizes",
    hdrs = [
        "sizes.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "implementations_test",
    srcs = [
        "implementations_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":implementations",
        ":logging",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "printf_formats",
    hdrs = [
        "printf_formats.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
    ],
)

cc_library(
    name = "implementations",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logging",
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
