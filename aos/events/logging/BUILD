load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")

flatbuffer_cc_library(
    name = "logger_fbs",
    srcs = ["logger.fbs"],
    gen_reflections = True,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "replay_timing_fbs",
    srcs = ["replay_timing.fbs"],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_static_flatbuffer(
    name = "replay_timing_schema",
    function = "aos::timing::ReplayTimingSchema",
    target = ":replay_timing_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "replay_config_fbs",
    srcs = ["log_replayer_config.fbs"],
    gen_reflections = True,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "boot_timestamp",
    srcs = ["boot_timestamp.cc"],
    hdrs = ["boot_timestamp.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
    ],
)

cc_binary(
    name = "log_replayer",
    srcs = [
        "log_replayer.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        ":log_reader_utils",
        ":replay_config_fbs",
        ":replay_timing_fbs",
        ":replay_timing_schema",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "log_reader_utils",
    srcs = [
        "log_reader_utils.cc",
    ],
    hdrs = [
        "log_reader_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:uuid",
        "//aos/containers:resizeable_buffer",
        "//aos/events:event_loop",
        "//aos/util:file",
        "@com_google_absl//absl/strings",
    ],
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
    copts = select({
        "//tools:cpu_k8": ["-DENABLE_S3=1"],
        "//conditions:default": ["-DENABLE_S3=0"],
    }),
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":boot_timestamp",
        ":snappy_encoder",
        ":buffer_encoder",
        ":logger_fbs",
        "//aos:uuid",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos/containers:resizeable_buffer",
        "//aos/events:event_loop",
        "//aos/network:remote_message_fbs",
        "//aos/util:file",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
        "@boringssl//:crypto",
    ] + select({
        "//tools:cpu_k8": [
            ":s3_fetcher",
            ":lzma_encoder",
        ],
        "//tools:cpu_arm64": [":lzma_encoder"],
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
        "//aos:flatbuffers",
        "//aos/containers:resizeable_buffer",
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
    name = "snappy_encoder",
    srcs = [
        "snappy_encoder.cc",
    ],
    hdrs = [
        "snappy_encoder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":logger_fbs",
        "//aos:configuration_fbs",
        "//aos/containers:resizeable_buffer",
        "//aos/util:crc32",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
        "@snappy",
    ],
)

cc_test(
    name = "snappy_encoder_test",
    srcs = [
        "snappy_encoder_test.cc",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":buffer_encoder_param_test",
        ":snappy_encoder",
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

cc_library(
    name = "s3_fetcher",
    srcs = [
        "s3_fetcher.cc",
    ],
    hdrs = [
        "s3_fetcher.h",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        "//aos/containers:resizeable_buffer",
        "@aws_sdk//:core",
        "@aws_sdk//:s3",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "lzma_encoder_test",
    srcs = [
        "lzma_encoder_test.cc",
    ],
    shard_count = 8,
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
        "//aos:uuid",
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
        ":replay_timing_fbs",
        "//aos:condition",
        "//aos:uuid",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/mutex",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:multinode_timestamp_filter",
        "//aos/network:remote_message_fbs",
        "//aos/network:remote_message_schema",
        "//aos/network:team_number",
        "//aos/network:timestamp_filter",
        "//aos/time",
        "//aos/util:file",
        "//aos/util:threaded_queue",
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
        "//aos:aos_cli_utils",
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
    name = "single_node_merge",
    srcs = [
        "single_node_merge.cc",
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
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_writer",
        ":snappy_encoder",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging:log_namer",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "multinode_logger_test_lib",
    testonly = True,
    srcs = [
        "multinode_logger_test_lib.cc",
    ],
    hdrs = ["multinode_logger_test_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        ":log_writer",
        ":snappy_encoder",
        "//aos/events:message_counter",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
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
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split3_config",
    src = "multinode_pingpong_split3.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split4_config",
    src = "multinode_pingpong_split4.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split4_reliable_config",
    src = "multinode_pingpong_split4_reliable.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
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
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_triangle_split_config",
    src = "multinode_pingpong_triangle_split.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

cc_test(
    name = "realtime_replay_test",
    srcs = ["realtime_replay_test.cc"],
    data = [
        ":multinode_pingpong_combined_config",
        "//aos/events:pingpong_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        ":log_writer",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_test(
    name = "log_reader_utils_test",
    srcs = ["log_reader_utils_test.cc"],
    data = [
        ":multinode_pingpong_combined_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader_utils",
        ":multinode_logger_test_lib",
    ],
)

cc_test(
    name = "multinode_logger_test",
    srcs = ["multinode_logger_test.cc"],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    data = [
        ":multinode_pingpong_combined_config",
        ":multinode_pingpong_split3_config",
        ":multinode_pingpong_split4_config",
        ":multinode_pingpong_split4_reliable_config",
        ":multinode_pingpong_split_config",
        ":multinode_pingpong_triangle_split_config",
        "//aos/events:pingpong_config",
    ],
    shard_count = 10,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":multinode_logger_test_lib",
    ],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cc"],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    data = [
        "//aos/events:pingpong_config",
    ],
    shard_count = 10,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        ":log_writer",
        ":snappy_encoder",
        "//aos/events:message_counter",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_test(
    name = "logfile_utils_test",
    srcs = ["logfile_utils_test.cc"],
    data = [
        ":logger_fbs_reflection_out",
        "//aos/network:remote_message_fbs_reflection_out",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        ":test_message_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//aos/testing:tmpdir",
        "@com_github_google_flatbuffers//src:flatc_library",
    ],
)

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "timestamp_plot",
    srcs = ["timestamp_plot.cc"],
    deps = [
        "//aos:init",
        "//frc971/analysis:in_process_plotter",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "log_config_extractor",
    srcs = [
        "log_config_extractor.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:aos_cli_utils",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events/logging:log_reader",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)
