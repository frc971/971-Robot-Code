package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "dump_rtprio",
    srcs = [
        "dump_rtprio.cc",
    ],
    deps = [
        "//aos/common:time",
        "//aos/common/logging",
        "//aos/common/logging:implementations",
    ],
)

cc_library(
    name = "queue",
    hdrs = [
        "queue-tmpl.h",
    ],
    visibility = ["//aos/common:__pkg__"],
)

cc_library(
    name = "complex_thread_local",
    srcs = [
        "complex_thread_local.cc",
    ],
    hdrs = [
        "complex_thread_local.h",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        "//aos:once",
        "//aos/common:die",
    ],
)

cc_test(
    name = "complex_thread_local_test",
    srcs = [
        "complex_thread_local_test.cc",
    ],
    deps = [
        ":complex_thread_local",
        "//aos/common/logging",
        "//aos/common/util:thread",
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
    deps = [
        "//aos/common:die",
        "//aos/common/logging:implementations",
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
    deps = [
        "//aos:once",
        "//aos/common:unique_malloc_ptr",
        "//aos/common/logging",
    ],
)

cc_binary(
    name = "core",
    srcs = [
        "core.cc",
    ],
    deps = [
        ":init",
        "//aos/common/util:run_command",
    ],
)
