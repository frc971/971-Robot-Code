load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

flatbuffer_cc_library(
    name = "logger_fbs",
    srcs = ["logger.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "logfile_utils",
    srcs = [
        "logfile_sorting.cc",
        "logfile_utils.cc",
    ],
    hdrs = [
        "logfile_sorting.h",
        "logfile_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":logger_fbs",
        ":uuid",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos/containers:resizeable_buffer",
        "//aos/events:event_loop",
        "//aos/util:file",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
        "@boringssl//:crypto",
    ] + select({
        "//tools:cpu_k8": [":lzma_encoder"],
        "//tools:cpu_aarch64": [":lzma_encoder"],
        "//conditions:default": [],
    }),
)

cc_binary(
    name = "logfile_utils_out_of_space_test_runner",
    testonly = True,
    srcs = [
        "logfile_utils_out_of_space_test_runner.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        "//aos:init",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

sh_test(
    name = "logfile_utils_out_of_space_test",
    srcs = [
        "logfile_utils_out_of_space_test.sh",
    ],
    data = [
        ":logfile_utils_out_of_space_test_runner",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

cc_library(
    name = "buffer_encoder",
    srcs = [
        "buffer_encoder.cc",
    ],
    hdrs = [
        "buffer_encoder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logger_fbs",
        "//aos:configuration_fbs",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "buffer_encoder_test",
    srcs = [
        "buffer_encoder_test.cc",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":buffer_encoder",
        ":buffer_encoder_param_test",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "lzma_encoder",
    srcs = [
        "lzma_encoder.cc",
    ],
    hdrs = [
        "lzma_encoder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":logger_fbs",
        "//aos:configuration_fbs",
        "//aos/containers:resizeable_buffer",
        "//third_party:lzma",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "lzma_encoder_test",
    srcs = [
        "lzma_encoder_test.cc",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":buffer_encoder_param_test",
        ":lzma_encoder",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "buffer_encoder_param_test",
    testonly = True,
    srcs = [
        "buffer_encoder_param_test.cc",
    ],
    hdrs = [
        "buffer_encoder_param_test.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "log_namer",
    srcs = [
        "log_namer.cc",
    ],
    hdrs = [
        "log_namer.h",
    ],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        ":uuid",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "log_writer",
    srcs = [
        "log_writer.cc",
    ],
    hdrs = [
        "log_writer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":log_namer",
        "//aos:configuration",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/network:message_bridge_server_fbs",
    ],
)

cc_library(
    name = "log_reader",
    srcs = [
        "log_reader.cc",
    ],
    hdrs = [
        "log_reader.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_namer",
        ":log_writer",
        ":logfile_utils",
        ":logger_fbs",
        ":uuid",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:multinode_timestamp_filter",
        "//aos/network:remote_message_fbs",
        "//aos/network:remote_message_schema",
        "//aos/network:team_number",
        "//aos/network:timestamp_filter",
        "//aos/time",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "log_cat",
    srcs = [
        "log_cat.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "timestamp_extractor",
    srcs = [
        "timestamp_extractor.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/network:multinode_timestamp_filter",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "log_edit",
    srcs = [
        "log_edit.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/util:file",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "log_stats",
    srcs = [
        "log_stats.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/time",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "logger_main",
    srcs = [
        "logger_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_writer",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging:log_namer",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

aos_config(
    name = "multinode_pingpong_split_config",
    src = "multinode_pingpong_split.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:config"],
)

aos_config(
    name = "multinode_pingpong_combined_config",
    src = "multinode_pingpong_combined.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:config"],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cc"],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_aarch64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    data = [
        ":multinode_pingpong_combined_config",
        ":multinode_pingpong_split_config",
        "//aos/events:pingpong_config",
    ],
    shard_count = 5,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        ":log_writer",
        "//aos/events:message_counter",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
    ],
)

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
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

cc_test(
    name = "logfile_utils_test",
    srcs = ["logfile_utils_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        ":test_message_fbs",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
    ],
)

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)
