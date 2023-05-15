load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_py_library", "flatbuffer_rust_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("@rules_rust//rust:defs.bzl", "rust_library", "rust_test")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")

exports_files(["aos_dump_autocomplete.sh"])

filegroup(
    name = "prime_binaries",
    srcs = [
        "//aos:aos_dump",
        "//aos:aos_dump_autocomplete.sh",
        "//aos:aos_send",
        "//aos/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries",
    srcs = [
        "//aos/events/logging:logger_main",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_binaries_stripped",
    srcs = [
        "//aos:aos_dump.stripped",
        "//aos:aos_dump_autocomplete.sh",
        "//aos:aos_send.stripped",
        "//aos/starter:starter_stripped",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries_stripped",
    srcs = [
        "//aos/events/logging:logger_main.stripped",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "math",
    hdrs = [
        "commonmath.h",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "macros",
    hdrs = [
        "macros.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtest_prod",
    hdrs = [
        "gtest_prod.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "unique_malloc_ptr",
    hdrs = [
        "unique_malloc_ptr.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":die",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "dump_rtprio",
    srcs = [
        "dump_rtprio.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "//aos/util:top",
        "@com_github_google_glog//:glog",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":realtime",
        "//aos:die",
        "//aos/logging:implementations",
    ],
)

autocxx_library(
    name = "init_rs",
    srcs = ["init.rs"],
    crate_name = "aos_init",
    libs = [
        ":init",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "realtime",
    srcs = [
        "realtime.cc",
    ],
    hdrs = [
        "realtime.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":thread_local",
        "@com_github_google_glog//:glog",
    ],
)

flatbuffer_cc_library(
    name = "configuration_fbs",
    srcs = ["configuration.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_static_flatbuffer(
    name = "configuration_schema",
    function = "aos::ConfigurationSchema",
    target = ":configuration_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "configuration_ts_fbs",
    srcs = ["configuration.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "configuration_fbs_python",
    srcs = ["configuration.fbs"],
    namespace = "aos",
    tables = [
        "Configuration",
        "Channel",
        "Connection",
        "Map",
        "Node",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_rust_library(
    name = "configuration_rust_fbs",
    srcs = ["configuration.fbs"],
    crate_name = "aos_configuration_fbs",
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
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
    target_compatible_with = ["@platforms//os:linux"],
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

cc_library(
    name = "configuration_for_rust",
    srcs = [
        "configuration_for_rust.cc",
    ],
    hdrs = [
        "configuration_for_rust.h",
    ],
    deps = [
        ":configuration",
        ":for_rust",
        "//third_party/cargo:cxx_cc",
    ],
)

autocxx_library(
    name = "configuration_rs",
    srcs = ["configuration.rs"],
    crate_name = "aos_configuration",
    libs = [
        ":configuration",
        ":configuration_for_rust",
        ":configuration_fbs",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
    deps = [
        ":configuration_rust_fbs",
        ":flatbuffers_rs",
        "//third_party/cargo:thiserror",
    ],
)

rust_test(
    name = "configuration_rs_test",
    crate = ":configuration_rs",
    data = [
        "//aos/testdata:test_configs",
    ],
    # TODO: Make Rust play happy with pic vs nopic. Details at:
    # https://github.com/bazelbuild/rules_rust/issues/118
    rustc_flags = ["-Crelocation-model=static"],
)

flatbuffer_ts_library(
    name = "json_to_flatbuffer_fbs_ts",
    srcs = ["json_to_flatbuffer.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
)

flatbuffer_cc_library(
    name = "json_to_flatbuffer_fbs",
    srcs = ["json_to_flatbuffer.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
)

flatbuffer_rust_library(
    name = "json_to_flatbuffer_rust_fbs",
    srcs = ["json_to_flatbuffer.fbs"],
    crate_name = "aos_json_to_flatbuffer_fbs",
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
    visibility = ["//aos:__subpackages__"],
)

cc_library(
    name = "flatbuffer_utils",
    srcs = ["flatbuffer_utils.cc"],
    hdrs = ["flatbuffer_utils.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "json_tokenizer",
    srcs = ["json_tokenizer.cc"],
    hdrs = ["json_tokenizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":fast_string_builder",
        ":flatbuffer_utils",
        ":flatbuffers",
        ":json_tokenizer",
        "//aos/util:file",
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
    data = [
        ":json_to_flatbuffer_fbs_reflection_out",
        ":json_to_flatbuffer_test_spaces.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flatbuffer_merge",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_test(
    name = "flatbuffer_introspection_test",
    srcs = [
        "flatbuffer_introspection_test.cc",
    ],
    data = [
        ":json_to_flatbuffer_fbs_reflection_out",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "flatbuffer_merge",
    srcs = ["flatbuffer_merge.cc"],
    hdrs = ["flatbuffer_merge.h"],
    copts = ["-Wno-cast-align"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flatbuffer_merge",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
        "//aos/containers:resizeable_buffer",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

rust_library(
    name = "flatbuffers_rs",
    srcs = ["flatbuffers.rs"],
    crate_name = "aos_flatbuffers",
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//rust",
    ],
)

rust_test(
    name = "flatbuffers_rs_test",
    crate = ":flatbuffers_rs",
    deps = [
        ":json_to_flatbuffer_rust_fbs",
    ],
)

cc_test(
    name = "configuration_test",
    srcs = [
        "configuration_test.cc",
    ],
    data = [
        "//aos/events:ping_fbs_reflection_out",
        "//aos/events:pingpong_config",
        "//aos/events:pong_fbs_reflection_out",
        "//aos/testdata:test_configs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":configuration",
        "//aos/events:ping_fbs",
        "//aos/testing:flatbuffer_eq",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:test_logging",
    ],
)

cc_binary(
    name = "config_flattener",
    srcs = [
        "config_flattener.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        ":init",
        "//aos/util:file",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "aos_cli_utils",
    srcs = [
        "aos_cli_utils.cc",
    ],
    hdrs = [
        "aos_cli_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "aos_dump",
    srcs = [
        "aos_dump.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_cli_utils",
        ":configuration",
        ":json_to_flatbuffer",
        "//aos:init",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "aos_send",
    srcs = [
        "aos_send.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_cli_utils",
        ":configuration",
        ":init",
        ":json_to_flatbuffer",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "aos_graph_nodes",
    srcs = [
        "aos_graph_nodes.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        ":json_to_flatbuffer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "ftrace",
    srcs = [
        "ftrace.cc",
    ],
    hdrs = [
        "ftrace.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "fast_string_builder",
    srcs = [
        "fast_string_builder.cc",
    ],
    hdrs = [
        "fast_string_builder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "thread_local",
    hdrs = [
        "thread_local.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "realtime_test",
    srcs = [
        "realtime_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":init",
        ":realtime",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "flatbuffers_test",
    srcs = [
        "flatbuffers_test.cc",
    ],
    deps = [
        ":flatbuffers",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
    ],
)

py_binary(
    name = "flatbuffers_static",
    srcs = ["flatbuffers_static.py"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":thread_local",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "uuid_test",
    srcs = ["uuid_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":uuid",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "uuid_for_rust",
    hdrs = ["uuid_for_rust.h"],
    deps = [
        ":uuid",
    ],
)

autocxx_library(
    name = "uuid_rs",
    srcs = ["uuid.rs"],
    crate_name = "aos_uuid",
    libs = [
        ":uuid",
        ":uuid_for_rust",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "//third_party/cargo:uuid",
    ],
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "aos_graph_channels",
    srcs = [
        "aos_graph_channels.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/time",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "for_rust",
    hdrs = [
        "for_rust.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/cargo:cxx_cc",
    ],
)

cc_library(
    name = "sha256",
    srcs = [
        "sha256.cc",
    ],
    hdrs = ["sha256.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@boringssl//:crypto",
        "@com_google_absl//absl/types:span",
    ],
)
