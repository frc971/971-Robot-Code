package(default_visibility = ["//visibility:public"])

cc_library(
    name = "aos_sync",
    srcs = [
        "aos_sync.cc",
    ],
    hdrs = [
        "aos_sync.h",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        "//aos:macros",
        "//aos:once",
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
    linkopts = [
        "-lrt",
    ],
    deps = [
        ":core_lib",
        ":shared_mem",
        "//aos:condition",
        "//aos/logging",
        "//aos/mutex",
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
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_shm",
        "//aos/time",
        "//aos/util:death_test_log_implementation",
        "//aos/util:thread",
    ],
)

cc_test(
    name = "ipc_stress_test",
    srcs = [
        "ipc_stress_test.cc",
    ],
    tags = [
        "manual",
    ],
    deps = [
        ":core_lib",
        "//aos:die",
        "//aos/libc:aos_strsignal",
        "//aos/libc:dirname",
        "//aos/logging",
        "//aos/mutex",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
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
        "//aos:init",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/mutex",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "signalfd",
    srcs = [
        "signalfd.cc",
    ],
    hdrs = [
        "signalfd.h",
    ],
    deps = [
        "//aos/logging",
    ],
)

cc_library(
    name = "index",
    srcs = ["index.cc"],
    hdrs = ["index.h"],
)

cc_test(
    name = "index_test",
    srcs = ["index_test.cc"],
    deps = [
        ":index",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "lockless_queue",
    srcs = [
        "lockless_queue.cc",
        "lockless_queue_memory.h",
    ],
    hdrs = ["lockless_queue.h"],
    deps = [
        ":aos_sync",
        ":index",
        "//aos:init",
        "//aos/logging",
        "//aos/time",
        "//aos/util:compiler_memory_barrier",
    ],
)

cc_library(
    name = "queue_racer",
    testonly = True,
    srcs = [
        "queue_racer.cc",
    ],
    hdrs = [
        "queue_racer.h",
    ],
    deps = [
        ":lockless_queue",
        "//aos:event",
        "//third_party/googletest:gtest",
    ],
)

cc_test(
    name = "lockless_queue_test",
    timeout = "eternal",
    srcs = ["lockless_queue_test.cc"],
    deps = [
        ":lockless_queue",
        ":queue_racer",
        ":signalfd",
        "//aos:event",
        "//aos/events:epoll",
        "//aos/libc:aos_strsignal",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_logging",
    ],
)

cc_test(
    name = "lockless_queue_death_test",
    srcs = ["lockless_queue_death_test.cc"],
    deps = [
        ":lockless_queue",
        ":queue_racer",
        ":signalfd",
        "//aos:event",
        "//aos/events:epoll",
        "//aos/libc:aos_strsignal",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_logging",
    ],
)
