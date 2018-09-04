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
    compatible_with = [
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":sizes",
        "//aos/common:die",
        "//aos/common:macros",
        "//aos/common/libc:aos_strerror",
        "//aos/linux_code:complex_thread_local",
    ],
)

cc_library(
    name = "replay",
    srcs = [
        "replay.cc",
    ],
    hdrs = [
        "replay.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":binary_log_file",
        ":logging",
        "//aos/common:queues",
        "//aos/linux_code/ipc_lib:queue",
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
        ":logging",
        "//aos/common:die",
        "//aos/common:queue_types",
        "//aos/common:time",
        "//aos/linux_code:configuration",
        "//aos/linux_code:init",
        "//aos/linux_code/ipc_lib:queue",
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
        "//aos/common:time",
        "//aos/linux_code:init",
        "//aos/linux_code/ipc_lib:queue",
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
        "//aos/common:queue_types",
        "//aos/common/util:string_to_num",
        "//aos/linux_code:configuration",
        "//aos/linux_code:init",
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
    compatible_with = [
        "//tools:armhf-debian",
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
    name = "queue_logging",
    srcs = [
        "queue_logging.cc",
    ],
    hdrs = [
        "queue_logging.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logging",
        ":sizes",
        "//aos/common:die",
        "//aos/common:queue_types",
    ],
)

cc_library(
    name = "matrix_logging",
    srcs = [
        "matrix_logging.cc",
    ],
    hdrs = [
        "matrix_logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logging",
        ":sizes",
        "//aos/common:die",
        "//aos/common:generated_queue_headers",
        "//aos/common:queue_types",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "printf_formats",
    hdrs = [
        "printf_formats.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/common:macros",
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
    compatible_with = [
        "//tools:armhf-debian",
    ],
    linkopts = [
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logging",
        ":sizes",
        "//aos:once",
        "//aos/common:die",
        "//aos/common:macros",
        "//aos/common:mutex",
        "//aos/common:queue_types",
        "//aos/common:time",
        "//aos/common:type_traits",
        "//aos/linux_code/ipc_lib:queue",
    ],
)
