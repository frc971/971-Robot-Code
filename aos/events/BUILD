load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "event_loop_fbs",
    srcs = ["event_loop.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "ping_fbs",
    srcs = ["ping.fbs"],
    gen_reflections = 1,
)

flatbuffer_ts_library(
    name = "ping_ts_fbs",
    srcs = ["ping.fbs"],
)

flatbuffer_cc_library(
    name = "pong_fbs",
    srcs = ["pong.fbs"],
    gen_reflections = 1,
)

cc_library(
    name = "epoll",
    srcs = ["epoll.cc"],
    hdrs = ["epoll.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "epoll_test",
    srcs = ["epoll_test.cc"],
    deps = [
        ":epoll",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "event_loop",
    srcs = [
        "event_loop.cc",
        "event_loop_event.h",
        "event_loop_tmpl.h",
    ],
    hdrs = [
        "channel_preallocated_allocator.h",
        "event_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_fbs",
        ":timing_statistics",
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos:flatbuffers",
        "//aos:ftrace",
        "//aos/ipc_lib:data_alignment",
        "//aos/logging:implementations",
        "//aos/time",
        "//aos/util:phased_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/container:btree",
    ],
)

cc_library(
    name = "ping_lib",
    srcs = [
        "ping_lib.cc",
    ],
    hdrs = [
        "ping_lib.h",
    ],
    deps = [
        ":event_loop",
        ":ping_fbs",
        ":pong_fbs",
        "//aos:json_to_flatbuffer",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "ping",
    srcs = [
        "ping.cc",
    ],
    data = [":pingpong_config"],
    deps = [
        ":ping_lib",
        ":shm_event_loop",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_github_google_glog//:glog",
    ],
)

aos_config(
    name = "config",
    src = "aos.json",
    flatbuffers = [
        ":event_loop_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
    ],
)

aos_config(
    name = "pingpong_config",
    src = "pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
    ],
    deps = [":config"],
)

aos_config(
    name = "multinode_pingpong_config",
    src = "multinode_pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/events/logging:logger_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:message_bridge_server_fbs",
    ],
    deps = [":config"],
)

cc_library(
    name = "pong_lib",
    srcs = [
        "pong_lib.cc",
    ],
    hdrs = [
        "pong_lib.h",
    ],
    deps = [
        ":event_loop",
        ":ping_fbs",
        ":pong_fbs",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "pong",
    srcs = [
        "pong.cc",
    ],
    data = [":pingpong_config"],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        ":pong_lib",
        ":shm_event_loop",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "pingpong_test",
    srcs = ["pingpong_test.cc"],
    data = [":pingpong_config"],
    deps = [
        ":ping_lib",
        ":pong_lib",
        ":simulated_event_loop",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "timing_statistics",
    srcs = ["timing_statistics.cc"],
    hdrs = ["timing_statistics.h"],
    deps = [
        ":event_loop_fbs",
        "//aos:configuration",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "timing_statistics_test",
    srcs = ["timing_statistics_test.cc"],
    deps = [
        ":timing_statistics",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "shm_event_loop",
    srcs = ["shm_event_loop.cc"],
    hdrs = ["shm_event_loop.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":event_loop_fbs",
        ":test_message_fbs",
        ":timing_statistics",
        "//aos:realtime",
        "//aos/ipc_lib:lockless_queue",
        "//aos/ipc_lib:signalfd",
        "//aos/stl_mutex",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/base",
    ],
)

cc_test(
    name = "shm_event_loop_test",
    srcs = ["shm_event_loop_test.cc"],
    shard_count = 16,
    deps = [
        ":event_loop_param_test",
        ":shm_event_loop",
        ":test_message_fbs",
    ],
)

cc_library(
    name = "event_loop_param_test",
    testonly = True,
    srcs = ["event_loop_param_test.cc"],
    hdrs = ["event_loop_param_test.h"],
    deps = [
        ":event_loop",
        ":test_message_fbs",
        "//aos:realtime",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "simulated_event_loop_test",
    srcs = ["simulated_event_loop_test.cc"],
    data = [":multinode_pingpong_config"],
    shard_count = 4,
    deps = [
        ":event_loop_param_test",
        ":message_counter",
        ":ping_lib",
        ":pong_lib",
        ":simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "simple_channel",
    srcs = ["simple_channel.cc"],
    hdrs = ["simple_channel.h"],
    deps = [
        "//aos:configuration_fbs",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "simulated_event_loop",
    srcs = [
        "event_scheduler.cc",
        "simulated_event_loop.cc",
        "simulated_network_bridge.cc",
    ],
    hdrs = [
        "event_scheduler.h",
        "simulated_event_loop.h",
        "simulated_network_bridge.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":event_loop",
        ":simple_channel",
        "//aos:realtime",
        "//aos/events/logging:logger_fbs",
        "//aos/ipc_lib:index",
        "//aos/network:message_bridge_client_status",
        "//aos/network:message_bridge_server_status",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/container:btree",
    ],
)

cc_test(
    name = "event_scheduler_test",
    srcs = ["event_scheduler_test.cc"],
    deps = [
        ":simulated_event_loop",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "aos_logging",
    srcs = [
        "aos_logging.cc",
    ],
    hdrs = [
        "aos_logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
        "//aos/logging:implementations",
        "//aos/logging:log_message_fbs",
    ],
)

cc_library(
    name = "message_counter",
    hdrs = ["message_counter.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
    ],
)
