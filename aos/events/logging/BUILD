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
        ":logger_fbs",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//aos/util:file",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
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
    deps = [
        ":logger",
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
