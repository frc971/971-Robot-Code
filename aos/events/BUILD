load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "pingpong_fbs",
    srcs = ["pingpong.fbs"],
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

cc_library(
    name = "event_loop",
    srcs = ["event_loop_tmpl.h"],
    hdrs = [
        "event_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos:flatbuffers",
        "//aos/time",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_binary(
    name = "ping",
    srcs = [
        "ping.cc",
    ],
    data = ["config.fb.json"],
    deps = [
        ":pingpong_fbs",
        ":shm_event_loop",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "pong",
    srcs = [
        "pong.cc",
    ],
    data = ["config.fb.json"],
    deps = [
        ":pingpong_fbs",
        ":shm_event_loop",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "shm_event_loop",
    srcs = ["shm_event_loop.cc"],
    hdrs = ["shm_event_loop.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":epoll",
        ":event_loop",
        ":test_message_fbs",
        "//aos:realtime",
        "//aos/ipc_lib:lockless_queue",
        "//aos/ipc_lib:signalfd",
        "//aos/util:phased_loop",
    ],
)

cc_test(
    name = "shm_event_loop_test",
    srcs = ["shm_event_loop_test.cc"],
    shard_count = 5,
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
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "simulated_event_loop_test",
    srcs = ["simulated_event_loop_test.cc"],
    deps = [
        ":event_loop_param_test",
        ":simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "simulated_event_loop",
    testonly = True,
    srcs = [
        "event_scheduler.cc",
        "simulated_event_loop.cc",
    ],
    hdrs = [
        "event_scheduler.h",
        "simulated_event_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
        "//aos/ipc_lib:index",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/container:btree",
    ],
)
