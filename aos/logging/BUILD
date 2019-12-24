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

cc_binary(
    name = "binary_log_writer",
    srcs = [
        "binary_log_writer.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":binary_log_file",
        ":implementations",
        ":log_namer",
        ":logging",
        "//aos:configuration",
        "//aos:die",
        "//aos:init",
        "//aos/ipc_lib:queue",
        "//aos/time",
    ],
)

cc_binary(
    name = "log_streamer",
    srcs = [
        "log_streamer.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":implementations",
        ":logging",
        "//aos:init",
        "//aos/ipc_lib:queue",
        "//aos/time",
    ],
)

cc_binary(
    name = "log_displayer",
    srcs = [
        "log_displayer.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":binary_log_file",
        ":implementations",
        ":logging",
        "//aos:configuration",
        "//aos:init",
        "//aos/util:string_to_num",
    ],
)

cc_library(
    name = "binary_log_file",
    srcs = [
        "binary_log_file.cc",
    ],
    hdrs = [
        "binary_log_file.h",
    ],
    deps = [
        ":implementations",
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
