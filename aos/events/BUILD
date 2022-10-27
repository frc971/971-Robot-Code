load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_rust_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos:config.bzl", "aos_config")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")
load("@rules_rust//rust:defs.bzl", "rust_doc_test", "rust_test")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_static_flatbuffer(
    name = "test_message_schema",
    function = "aos::TestMessageSchema",
    target = ":test_message_fbs_reflection_out",
)

flatbuffer_cc_library(
    name = "event_loop_fbs",
    srcs = ["event_loop.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos:configuration_fbs"],
)

cc_static_flatbuffer(
    name = "timing_report_schema",
    function = "aos::timing::ReportSchema",
    target = ":event_loop_fbs_reflection_out",
)

flatbuffer_cc_library(
    name = "ping_fbs",
    srcs = ["ping.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_ts_library(
    name = "ping_ts_fbs",
    srcs = ["ping.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_rust_library(
    name = "ping_rust_fbs",
    srcs = ["ping.fbs"],
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
)

flatbuffer_rust_library(
    name = "pong_rust_fbs",
    srcs = ["pong.fbs"],
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
)

flatbuffer_cc_library(
    name = "pong_fbs",
    srcs = ["pong.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "epoll",
    srcs = ["epoll.cc"],
    hdrs = ["epoll.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "epoll_test",
    srcs = ["epoll_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_fbs",
        ":timing_statistics",
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos:flatbuffers",
        "//aos:ftrace",
        "//aos:uuid",
        "//aos/ipc_lib:data_alignment",
        "//aos/logging:implementations",
        "//aos/time",
        "//aos/util:phased_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/container:btree",
    ],
)

cc_library(
    name = "event_loop_runtime_cc",
    srcs = [
        "event_loop_runtime.cc",
    ],
    hdrs = [
        "event_loop_runtime.h",
    ],
    deps = [
        ":event_loop",
        "//aos:for_rust",
        "//third_party/cargo:cxx_cc",
    ],
)

autocxx_library(
    name = "event_loop_runtime",
    srcs = ["event_loop_runtime.rs"],
    crate_name = "aos_events_event_loop_runtime",
    libs = [
        ":event_loop_runtime_cc",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "@com_github_google_flatbuffers//rust",
        "//third_party/cargo:uuid",
        "//third_party/cargo:futures",
        "//third_party/cargo:thiserror",
    ],
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_rs",
        "//aos:uuid_rs",
    ],
)

rust_doc_test(
    name = "event_loop_runtime_doc_test",
    crate = ":event_loop_runtime",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":pong_rust_fbs",
    ],
)

autocxx_library(
    name = "event_loop_runtime_test_lib_rs",
    testonly = True,
    srcs = ["event_loop_runtime_test_lib.rs"],
    libs = [
        ":event_loop",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        ":event_loop_runtime",
        ":ping_rust_fbs",
        ":pong_rust_fbs",
    ],
    target_compatible_with = select({
        "//conditions:default": ["//tools/platforms/rust:has_support"],
        "//tools:has_msan": ["@platforms//:incompatible"],
    }),
)

cc_test(
    name = "event_loop_runtime_test",
    srcs = ["event_loop_runtime_test.cc"],
    data = [":pingpong_config"],
    deps = [
        ":event_loop_runtime_test_lib_rs",
        ":ping_fbs",
        ":pong_fbs",
        ":simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "@com_github_google_glog//:glog",
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    name = "aos_config",
    src = "aos.json",
    flatbuffers = [
        ":event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "pingpong_config",
    src = "pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":aos_config"],
)

[
    aos_config(
        name = config + "_config",
        src = config + ".json",
        flatbuffers = [
            ":ping_fbs",
            ":pong_fbs",
            "//aos/network:message_bridge_client_fbs",
            "//aos/network:remote_message_fbs",
            "//aos/network:timestamp_fbs",
            "//aos/network:message_bridge_server_fbs",
        ],
        target_compatible_with = ["@platforms//os:linux"],
        deps = [":aos_config"],
    )
    for config in [
        "multinode_pingpong_test_split",
        "multinode_pingpong",
        "multinode_pingpong_test_combined",
    ]
]

cc_library(
    name = "pong_lib",
    srcs = [
        "pong_lib.cc",
    ],
    hdrs = [
        "pong_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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
    data = [
        ":multinode_pingpong_test_split_config",
        ":pingpong_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":ping_lib",
        ":pong_lib",
        ":simulated_event_loop",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_library(
    name = "timing_statistics",
    srcs = ["timing_statistics.cc"],
    hdrs = ["timing_statistics.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop_fbs",
        "//aos:configuration",
        "//aos/util:error_counter",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "timing_statistics_test",
    srcs = ["timing_statistics_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":event_loop_fbs",
        ":test_message_fbs",
        ":timing_statistics",
        "//aos:init",
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
    shard_count = 24,
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop",
        ":test_message_fbs",
        ":test_message_schema",
        ":timing_report_schema",
        "//aos:realtime",
        "//aos/logging:log_message_fbs",
        "//aos/logging:log_message_schema",
        "//aos/network:message_bridge_client_schema",
        "//aos/network:message_bridge_server_schema",
        "//aos/network:timestamp_schema",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "simulated_event_loop_test",
    srcs = ["simulated_event_loop_test.cc"],
    data = [
        ":multinode_pingpong_test_combined_config",
        ":multinode_pingpong_test_split_config",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop_param_test",
        ":message_counter",
        ":ping_lib",
        ":pong_lib",
        ":simulated_event_loop",
        "//aos/network:remote_message_fbs",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_library(
    name = "simple_channel",
    srcs = ["simple_channel.cc"],
    hdrs = ["simple_channel.h"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":simple_channel",
        "//aos:init",
        "//aos:realtime",
        "//aos/events/logging:boot_timestamp",
        "//aos/events/logging:logger_fbs",
        "//aos/ipc_lib:index",
        "//aos/network:message_bridge_client_status",
        "//aos/network:message_bridge_server_status",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_channel",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/container:btree",
    ],
)

cc_library(
    name = "simulated_event_loop_for_rust",
    hdrs = ["simulated_event_loop_for_rust.h"],
    deps = [
        ":simulated_event_loop",
        "//aos:for_rust",
        "//third_party/cargo:cxx_cc",
    ],
)

autocxx_library(
    name = "simulated_event_loop_rs",
    srcs = ["simulated_event_loop.rs"],
    crate_name = "aos_events_simulated_event_loop",
    libs = [
        ":simulated_event_loop",
        ":simulated_event_loop_for_rust",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "@com_github_google_flatbuffers//rust",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
        "//third_party/cargo:futures",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_runtime",
        "//aos:configuration_rs",
        "//aos:uuid_rs",
    ],
)

rust_test(
    name = "simulated_event_loop_rs_test",
    crate = ":simulated_event_loop_rs",
    data = [
        ":multinode_pingpong_test_combined_config",
    ],
    # TODO: Make Rust play happy with pic vs nopic. Details at:
    # https://github.com/bazelbuild/rules_rust/issues/118
    rustc_flags = ["-Crelocation-model=static"],
    deps = [
        ":ping_rust_fbs",
        "//aos:init_rs",
        "//third_party/cargo:futures",
        "@rules_rust//tools/runfiles",
    ],
)

cc_test(
    name = "event_scheduler_test",
    srcs = ["event_scheduler_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":simulated_event_loop",
        "//aos/network:testing_time_converter",
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
    ],
)

cc_library(
    name = "glib_main_loop",
    srcs = [
        "glib_main_loop.cc",
    ],
    hdrs = [
        "glib_main_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:shm_event_loop",
        "//third_party:gstreamer",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "glib_main_loop_test",
    srcs = [
        "glib_main_loop_test.cc",
    ],
    data = [
        ":aos_config",
    ],
    deps = [
        ":glib_main_loop",
        "//aos:configuration",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//third_party:gstreamer",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "timing_report_dump_lib",
    srcs = ["timing_report_dump_lib.cc"],
    hdrs = ["timing_report_dump_lib.h"],
    deps = ["//aos/events:event_loop"],
)

cc_binary(
    name = "timing_report_dump",
    srcs = ["timing_report_dump.cc"],
    deps = [
        ":timing_report_dump_lib",
        "//aos:init",
        "//aos/events/logging:log_reader",
    ],
)

cc_binary(
    name = "aos_timing_report_streamer",
    srcs = ["aos_timing_report_streamer.cc"],
    deps = [
        ":timing_report_dump_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
