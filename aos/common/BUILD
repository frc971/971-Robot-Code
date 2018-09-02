package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")
load("//tools:environments.bzl", "mcu_cpus")

queue_library(
    name = "test_queue",
    srcs = [
        "test_queue.q",
    ],
)

cc_library(
    name = "math",
    hdrs = [
        "commonmath.h",
    ],
    compatible_with = mcu_cpus,
)

cc_library(
    name = "macros",
    hdrs = [
        "macros.h",
    ],
    compatible_with = mcu_cpus + [
        "//tools:armhf-debian",
    ],
)

cc_library(
    name = "type_traits",
    hdrs = [
        "type_traits.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
)

cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":macros",
        ":mutex",
        "//aos/common/logging",
        "//aos/linux_code/ipc_lib:shared_mem",
    ],
)

genrule(
    name = "gen_queue_primitives",
    outs = ["queue_primitives.h"],
    cmd = "$(location //aos/build/queues:queue_primitives) $@",
    compatible_with = [
        "//tools:armhf-debian",
    ],
    tools = ["//aos/build/queues:queue_primitives"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "gen_print_field",
    outs = ["print_field.cc"],
    cmd = "$(location //aos/build/queues:print_field) $@",
    compatible_with = [
        "//tools:armhf-debian",
    ],
    tools = ["//aos/build/queues:print_field"],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "generated_queue_headers",
    hdrs = [
        ":gen_queue_primitives",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    visibility = ["//aos/common/logging:__pkg__"],
)

cc_library(
    name = "event",
    srcs = [
        "event.cc",
    ],
    hdrs = [
        "event.h",
    ],
    deps = [
        ":time",
        "//aos/common/logging",
        "//aos/linux_code/ipc_lib:aos_sync",
    ],
)

cc_library(
    name = "unique_malloc_ptr",
    hdrs = [
        "unique_malloc_ptr.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
)

cc_library(
    name = "queue_types",
    srcs = [
        "print_field_helpers.h",
        "queue_types.cc",
        ":gen_print_field",
    ],
    hdrs = [
        "queue_types.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":byteorder",
        ":generated_queue_headers",
        ":mutex",
        ":time",
        "//aos/common/logging:printf_formats",
        "//aos/linux_code/ipc_lib:core_lib",
        "//aos/linux_code/ipc_lib:shared_mem",
    ],
)

cc_test(
    name = "queue_types_test",
    srcs = [
        "queue_types_test.cc",
    ],
    deps = [
        ":queue_types",
        ":test_queue",
        "//aos/common/logging",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "network_port",
    hdrs = [
        "network_port.h",
    ],
)

cc_library(
    name = "byteorder",
    hdrs = [
        "byteorder.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
)

cc_library(
    name = "messages",
    srcs = [
        "message.cc",
    ],
    hdrs = [
        "message.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":byteorder",
        ":macros",
        ":time",
    ],
)

cc_library(
    name = "queues",
    hdrs = [
        "queue.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":messages",
        "//aos/linux_code:queue",
        "//aos/linux_code/ipc_lib:queue",
    ],
)

cc_library(
    name = "scoped_fd",
    hdrs = [
        "scoped_fd.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/common/logging",
    ],
)

cc_test(
    name = "queue_test",
    srcs = [
        "queue_test.cc",
    ],
    deps = [
        ":die",
        ":test_queue",
        "//aos/common/util:thread",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
    ],
)

cc_test(
    name = "type_traits_test",
    srcs = [
        "type_traits_test.cpp",
    ],
    deps = [
        ":type_traits",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "gtest_prod",
    hdrs = [
        "gtest_prod.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
)

cc_test(
    name = "time_test",
    srcs = [
        "time_test.cc",
    ],
    deps = [
        ":time",
        "//aos/common/logging",
        "//aos/common/util:death_test_log_implementation",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "die",
    srcs = [
        "die.cc",
    ],
    hdrs = [
        "die.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":macros",
        "//aos/common/libc:aos_strerror",
    ],
)

cc_test(
    name = "mutex_test",
    srcs = [
        "mutex_test.cc",
    ],
    deps = [
        ":die",
        ":mutex",
        "//aos/common:time",
        "//aos/common/logging",
        "//aos/common/util:death_test_log_implementation",
        "//aos/common/util:thread",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/testing:test_shm",
    ],
)

cc_test(
    name = "event_test",
    srcs = [
        "event_test.cc",
    ],
    deps = [
        ":event",
        ":time",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "condition",
    srcs = [
        "condition.cc",
    ],
    hdrs = [
        "condition.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":mutex",
        "//aos/common/logging",
        "//aos/linux_code/ipc_lib:aos_sync",
    ],
)

cc_test(
    name = "condition_test",
    srcs = [
        "condition_test.cc",
    ],
    deps = [
        ":condition",
        ":die",
        ":mutex",
        ":time",
        "//aos/common/logging",
        "//aos/common/util:thread",
        "//aos/linux_code/ipc_lib:aos_sync",
        "//aos/linux_code/ipc_lib:core_lib",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_shm",
    ],
)

cc_test(
    name = "die_test",
    srcs = [
        "die_test.cc",
    ],
    deps = [
        ":die",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "stl_mutex",
    hdrs = [
        "stl_mutex.h",
    ],
    deps = [
        "//aos/common/logging",
        "//aos/linux_code/ipc_lib:aos_sync",
    ],
)

cc_library(
    name = "mutex",
    srcs = [
        "mutex.cc",
    ],
    hdrs = [
        "mutex.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":die",
        ":type_traits",
        "//aos/common/logging",
        "//aos/linux_code/ipc_lib:aos_sync",
    ],
)

cc_test(
    name = "stl_mutex_test",
    srcs = [
        "stl_mutex_test.cc",
    ],
    deps = [
        ":die",
        ":stl_mutex",
        "//aos/common/util:thread",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "transaction",
    hdrs = [
        "transaction.h",
    ],
    deps = [
        "//aos/common/logging",
        "//aos/common/util:compiler_memory_barrier",
    ],
)

cc_test(
    name = "transaction_test",
    srcs = [
        "transaction_test.cc",
    ],
    deps = [
        ":transaction",
        "//aos/common/logging",
        "//aos/common/util:death_test_log_implementation",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "ring_buffer",
    hdrs = [
        "ring_buffer.h",
    ],
)

cc_test(
    name = "ring_buffer_test",
    srcs = [
        "ring_buffer_test.cc",
    ],
    deps = [
        ":ring_buffer",
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//aos:python_init"],
)
