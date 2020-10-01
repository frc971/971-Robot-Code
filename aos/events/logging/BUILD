load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

flatbuffer_cc_library(
    name = "logger_fbs",
    srcs = ["logger.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "logfile_utils",
    srcs = [
        "logfile_utils.cc",
    ],
    hdrs = [
        "logfile_utils.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":logger_fbs",
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
    deps = [
        ":buffer_encoder",
        ":buffer_encoder_param_test",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

# TODO(Brian): Properly restrict this to specific platforms and
# un-conditionalize the srcs and hdrs once we have support for that which
# interacts with select properly.
cc_library(
    name = "lzma_encoder",
    srcs = select({
        "//tools:cpu_k8": [
            "lzma_encoder.cc",
        ],
        "//tools:cpu_aarch64": [
            "lzma_encoder.cc",
        ],
        "//conditions:default": [],
    }),
    hdrs = select({
        "//tools:cpu_k8": [
            "lzma_encoder.h",
        ],
        "//tools:cpu_aarch64": [
            "lzma_encoder.h",
        ],
        "//conditions:default": [],
    }),
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
    restricted_to = [
        "//tools:k8",
        "//tools:aarch64",
    ],
    shard_count = 4,
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
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "logger",
    srcs = [
        "log_namer.cc",
        "logger.cc",
        "logger_math.cc",
    ],
    hdrs = [
        "eigen_mpq.h",
        "log_namer.h",
        "logger.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        ":uuid",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:team_number",
        "//aos/network:timestamp_filter",
        "//aos/time",
        "//aos/util:file",
        "//third_party/gmp",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "log_cat",
    srcs = [
        "log_cat.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logger",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "log_edit",
    srcs = [
        "log_edit.cc",
    ],
    deps = [
        ":logger",
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
    deps = [
        ":logger",
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
    visibility = ["//visibility:public"],
    deps = [
        ":logger",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging:log_namer",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

aos_config(
    name = "multinode_pingpong_config",
    src = "multinode_pingpong.json",
    flatbuffers = [
        ":logger_fbs",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:config"],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cc"],
    data = [
        ":multinode_pingpong_config",
        "//aos/events:pingpong_config",
    ],
    shard_count = 5,
    deps = [
        ":logger",
        "//aos/events:message_counter",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.h"],
)

cc_test(
    name = "uuid_test",
    srcs = ["uuid_test.cc"],
    deps = [
        ":uuid",
        "//aos/testing:googletest",
    ],
)
