load("//tools:environments.bzl", "mcu_cpus")
load("//aos/build:queues.bzl", "queue_library")

filegroup(
    name = "prime_binaries",
    srcs = [
        "//aos:core",
        "//aos/logging:log_displayer",
        "//aos/logging:log_streamer",
        "//aos/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries",
    srcs = [
        "//aos/logging:binary_log_writer",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_binaries_stripped",
    srcs = [
        # starter is hard coded to look for a non-stripped core...
        "//aos:core",
        "//aos/logging:log_streamer.stripped",
        "//aos/logging:log_displayer.stripped",
        "//aos/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries_stripped",
    srcs = [
        "//aos/logging:binary_log_writer.stripped",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "math",
    hdrs = [
        "commonmath.h",
    ],
    compatible_with = mcu_cpus,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "once",
    srcs = [
        "once-tmpl.h",
    ],
    hdrs = [
        "once.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:gtest_prod",
        "//aos/type_traits",
    ],
)

cc_library(
    name = "byteorder",
    hdrs = [
        "byteorder.h",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "once_test",
    srcs = [
        "once_test.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:once",
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "macros",
    hdrs = [
        "macros.h",
    ],
    compatible_with = mcu_cpus,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtest_prod",
    hdrs = [
        "gtest_prod.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "network_port",
    hdrs = [
        "network_port.h",
    ],
    visibility = ["//visibility:public"],
)

genrule(
    name = "gen_print_field",
    outs = ["print_field.cc"],
    cmd = "$(location //aos/build/queues:print_field) $@",
    tools = ["//aos/build/queues:print_field"],
    visibility = ["//visibility:private"],
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
    visibility = ["//visibility:public"],
    deps = [
        ":generated_queue_headers",
        "//aos:byteorder",
        "//aos/ipc_lib:core_lib",
        "//aos/ipc_lib:shared_mem",
        "//aos/logging:printf_formats",
        "//aos/mutex",
        "//aos/time",
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
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "queues",
    hdrs = [
        "queue.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:queue",
        "//aos/ipc_lib:queue",
        "//aos/messages",
    ],
)

queue_library(
    name = "test_queue",
    srcs = [
        "test_queue.q",
    ],
)

genrule(
    name = "gen_queue_primitives",
    outs = ["queue_primitives.h"],
    cmd = "$(location //aos/build/queues:queue_primitives) $@",
    tools = ["//aos/build/queues:queue_primitives"],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "generated_queue_headers",
    hdrs = [
        ":gen_queue_primitives",
    ],
    visibility = ["//aos/logging:__pkg__"],
)

cc_test(
    name = "queue_test",
    srcs = [
        "queue_test.cc",
    ],
    deps = [
        ":test_queue",
        "//aos:die",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/util:thread",
    ],
)

cc_library(
    name = "unique_malloc_ptr",
    hdrs = [
        "unique_malloc_ptr.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "condition",
    srcs = [
        "condition.cc",
    ],
    hdrs = [
        "condition.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/logging",
        "//aos/mutex",
    ],
)

cc_test(
    name = "condition_test",
    srcs = [
        "condition_test.cc",
    ],
    deps = [
        ":condition",
        "//aos:die",
        "//aos/ipc_lib:aos_sync",
        "//aos/ipc_lib:core_lib",
        "//aos/logging",
        "//aos/mutex",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_shm",
        "//aos/time",
        "//aos/util:thread",
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
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
        "//aos/libc:aos_strerror",
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
    name = "event",
    srcs = [
        "event.cc",
    ],
    hdrs = [
        "event.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/logging",
        "//aos/time",
    ],
)

cc_test(
    name = "event_test",
    srcs = [
        "event_test.cc",
    ],
    deps = [
        ":event",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
    ],
)

cc_binary(
    name = "dump_rtprio",
    srcs = [
        "dump_rtprio.cc",
    ],
    deps = [
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/time",
    ],
)

cc_library(
    name = "queue",
    hdrs = [
        "queue-tmpl.h",
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
    linkopts = [
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:die",
        "//aos:once",
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
        "//aos/testing:googletest",
        "//aos/util:thread",
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
    visibility = ["//visibility:public"],
    deps = [
        "//aos:die",
        "//aos/ipc_lib:shared_mem",
        "//aos/logging:implementations",
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
    visibility = ["//visibility:public"],
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

cc_library(
    name = "make_unique",
    hdrs = [
        "make_unique.h",
    ],
    visibility = ["//visibility:public"],
)
