package(default_visibility = ["//visibility:public"])

cc_library(
    name = "aos_sync",
    srcs = [
        "aos_sync.cc",
    ],
    hdrs = [
        "aos_sync.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        "//aos:once",
        "//aos:macros",
        "//aos/logging",
        "//aos/util:compiler_memory_barrier",
    ],
)

cc_library(
    name = "core_lib",
    srcs = [
        "core_lib.c",
    ],
    hdrs = [
        "core_lib.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":aos_sync",
        ":shared_mem_types",
    ],
)

cc_library(
    name = "shared_mem",
    srcs = [
        "shared_mem.c",
    ],
    hdrs = [
        "shared_mem.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    linkopts = [
        "-lrt",
    ],
    deps = [
        ":aos_sync",
        ":core_lib",
        ":shared_mem_types",
        "//aos/logging",
    ],
)

cc_library(
    # TODO(Brian): This should be shared_mem{,.h}, and the other one should be
    # shared_mem_init{,.cc,.h}.
    name = "shared_mem_types",
    hdrs = [
        "shared_mem_types.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":aos_sync",
    ],
)

cc_library(
    name = "queue",
    srcs = [
        "queue.cc",
    ],
    hdrs = [
        "queue.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    linkopts = [
        "-lrt",
    ],
    deps = [
        ":core_lib",
        ":shared_mem",
        "//aos:condition",
        "//aos/mutex:mutex",
        "//aos/logging",
        "//aos/util:options",
    ],
)

cc_test(
    name = "raw_queue_test",
    srcs = [
        "raw_queue_test.cc",
    ],
    deps = [
        ":core_lib",
        ":queue",
        "//aos:die",
        "//aos/time:time",
        "//aos/logging",
        "//aos/util:death_test_log_implementation",
        "//aos/util:thread",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_shm",
    ],
)

cc_test(
    name = "ipc_stress_test",
    srcs = [
        "ipc_stress_test.cc",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    tags = [
        "manual",
    ],
    deps = [
        ":core_lib",
        "//aos:die",
        "//aos/mutex:mutex",
        "//aos/time:time",
        "//aos/libc:aos_strsignal",
        "//aos/libc:dirname",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
    ],
)

cc_library(
    name = "scoped_message_ptr",
    deps = [
        ":queue",
    ],
)

cc_binary(
    name = "ipc_comparison",
    srcs = [
        "ipc_comparison.cc",
    ],
    deps = [
        ":queue",
        "//aos:condition",
        "//aos:event",
        "//aos/mutex:mutex",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/linux_code:init",
        "//third_party/gflags",
    ],
)
