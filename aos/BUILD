load("//tools:environments.bzl", "mcu_cpus")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_python_library")

filegroup(
    name = "prime_binaries",
    srcs = [
        "//aos:aos_dump",
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
        "//aos/events/logging:logger_main",
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
        "//aos:aos_dump.stripped",
        "//aos/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries_stripped",
    srcs = [
        "//aos/events/logging:logger_main.stripped",
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
        "//aos/mutex",
        "@com_github_google_glog//:glog",
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
        "//aos/time",
        "@com_github_google_glog//:glog",
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
    name = "complex_thread_local",
    srcs = [
        "complex_thread_local.cc",
    ],
    hdrs = [
        "complex_thread_local.h",
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
        ":realtime",
        "//aos:die",
        "//aos/ipc_lib:shared_mem",
        "//aos/logging:implementations",
    ],
)

cc_library(
    name = "realtime",
    srcs = [
        "realtime.cc",
    ],
    hdrs = [
        "realtime.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
    ],
)

flatbuffer_cc_library(
    name = "configuration_fbs",
    srcs = ["configuration.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_python_library(
    name = "configuration_fbs_python",
    srcs = ["configuration.fbs"],
    namespace = "aos",
    tables = [
        "Configuration",
        "Channel",
        "Map",
        "Node",
    ],
    visibility = ["//visibility:public"],
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
        ":configuration_fbs",
        ":flatbuffer_merge",
        ":flatbuffers",
        ":json_to_flatbuffer",
        "//aos:unique_malloc_ptr",
        "//aos/network:team_number",
        "//aos/util:file",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/strings",
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

flatbuffer_cc_library(
    name = "json_to_flatbuffer_flatbuffer",
    srcs = ["json_to_flatbuffer.fbs"],
    gen_reflections = 1,
)

cc_library(
    name = "flatbuffer_utils",
    srcs = ["flatbuffer_utils.cc"],
    hdrs = ["flatbuffer_utils.h"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "json_tokenizer",
    srcs = ["json_tokenizer.cc"],
    hdrs = ["json_tokenizer.h"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "json_to_flatbuffer",
    srcs = [
        "flatbuffer_introspection.cc",
        "json_to_flatbuffer.cc",
    ],
    hdrs = ["json_to_flatbuffer.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":flatbuffer_utils",
        ":flatbuffers",
        ":json_tokenizer",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "json_to_flatbuffer_test",
    srcs = [
        "json_to_flatbuffer_test.cc",
    ],
    deps = [
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_flatbuffer",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "flatbuffer_introspection_test",
    srcs = [
        "flatbuffer_introspection_test.cc",
    ],
    data = [
        ":json_to_flatbuffer_flatbuffer_reflection_out",
    ],
    deps = [
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_flatbuffer",
        "//aos/testing:googletest",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "flatbuffer_merge",
    srcs = ["flatbuffer_merge.cc"],
    hdrs = ["flatbuffer_merge.h"],
    copts = ["-Wno-cast-align"],
    visibility = ["//visibility:public"],
    deps = [
        ":flatbuffer_utils",
        ":flatbuffers",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_test(
    name = "flatbuffer_merge_test",
    srcs = [
        "flatbuffer_merge_test.cc",
    ],
    deps = [
        ":flatbuffer_merge",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_flatbuffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "flatbuffers",
    srcs = [
        "flatbuffers.cc",
    ],
    hdrs = [
        "flatbuffers.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "configuration_test",
    srcs = [
        "configuration_test.cc",
    ],
    data = [
        "testdata/backwards.json",
        "testdata/config1.json",
        "testdata/config1_bad.json",
        "testdata/config1_multinode.json",
        "testdata/config2.json",
        "testdata/config2_multinode.json",
        "testdata/config3.json",
        "testdata/expected.json",
        "testdata/expected_multinode.json",
        "testdata/good_multinode.json",
        "testdata/invalid_destination_node.json",
        "testdata/invalid_nodes.json",
        "testdata/invalid_source_node.json",
        "testdata/self_forward.json",
        "//aos/events:pingpong_config.json",
        "//aos/events:pong.bfbs",
    ],
    deps = [
        ":configuration",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_binary(
    name = "config_flattener",
    srcs = [
        "config_flattener.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        ":init",
        "//aos/util:file",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "aos_dump",
    srcs = [
        "aos_dump.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        ":json_to_flatbuffer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_github_google_glog//:glog",
    ],
)
