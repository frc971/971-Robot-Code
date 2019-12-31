load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

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
    name = "logger",
    srcs = [
        "logfile_utils.cc",
        "logger.cc",
    ],
    hdrs = [
        "logfile_utils.h",
        "logger.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logger_fbs",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/network:team_number",
        "//aos/time",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "log_cat",
    srcs = [
        "log_cat.cc",
    ],
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

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cc"],
    data = [
        "//aos/events:multinode_pingpong_config.json",
        "//aos/events:pingpong_config.json",
    ],
    deps = [
        ":logger",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
    ],
)
