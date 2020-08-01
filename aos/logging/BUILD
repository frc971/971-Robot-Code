load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

# The primary client logging interface.
cc_library(
    name = "logging",
    srcs = [
        "context.cc",
        "interface.cc",
    ],
    hdrs = [
        "context.h",
        "interface.h",
        "logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":sizes",
        "//aos:complex_thread_local",
        "//aos:die",
        "//aos:macros",
        "//aos/libc:aos_strerror",
    ],
)

cc_library(
    name = "log_namer",
    srcs = ["log_namer.cc"],
    hdrs = ["log_namer.h"],
    copts = ["-Wno-format-nonliteral"],
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
)

cc_test(
    name = "implementations_test",
    srcs = [
        "implementations_test.cc",
    ],
    deps = [
        ":implementations",
        ":logging",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "printf_formats",
    hdrs = [
        "printf_formats.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
    ],
)

cc_library(
    name = "implementations",
    srcs = [
        "implementations.cc",
    ],
    hdrs = [
        "implementations.h",
    ],
    linkopts = [
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logging",
        ":printf_formats",
        ":sizes",
        "//aos:die",
        "//aos:macros",
        "//aos/ipc_lib:queue",
        "//aos/mutex",
        "//aos/time",
        "//aos/type_traits",
        "@com_google_absl//absl/base",
    ],
)

flatbuffer_cc_library(
    name = "log_message_fbs",
    srcs = ["log_message.fbs"],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)
