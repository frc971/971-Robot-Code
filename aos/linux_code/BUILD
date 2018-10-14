package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "dump_rtprio",
    srcs = [
        "dump_rtprio.cc",
    ],
    deps = [
        "//aos/time:time",
        "//aos/logging",
        "//aos/logging:implementations",
    ],
)

cc_library(
    name = "queue",
    hdrs = [
        "queue-tmpl.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    visibility = ["//aos:__pkg__"],
)

cc_library(
    name = "complex_thread_local",
    srcs = [
        "complex_thread_local.cc",
    ],
    hdrs = [
        "complex_thread_local.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        "//aos:once",
        "//aos:die",
    ],
)

cc_test(
    name = "complex_thread_local_test",
    srcs = [
        "complex_thread_local_test.cc",
    ],
    deps = [
        ":complex_thread_local",
        "//aos/logging",
        "//aos/util:thread",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "init",
    srcs = [
        "init.cc",
    ],
    hdrs = [
        "init.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos:die",
        "//aos/logging:implementations",
        "//aos/linux_code/ipc_lib:shared_mem",
    ],
)

cc_library(
    name = "configuration",
    srcs = [
        "configuration.cc",
    ],
    hdrs = [
        "configuration.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos:once",
        "//aos:unique_malloc_ptr",
        "//aos/logging",
    ],
)

cc_binary(
    name = "core",
    srcs = [
        "core.cc",
    ],
    deps = [
        ":init",
        "//aos/util:run_command",
    ],
)
