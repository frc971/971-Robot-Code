load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "remote_message_fbs",
    srcs = ["remote_message.fbs"],
    gen_reflections = 1,
)

cc_static_flatbuffer(
    name = "remote_message_schema",
    function = "aos::message_bridge::RemoteMessageSchema",
    target = ":remote_message_fbs_reflection_out",
)

flatbuffer_cc_library(
    name = "connect_fbs",
    srcs = ["connect.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_ts_library(
    name = "connect_ts_fbs",
    srcs = ["connect.fbs"],
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "timestamp_fbs",
    srcs = ["timestamp.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "message_bridge_client_fbs",
    srcs = ["message_bridge_client.fbs"],
    gen_reflections = 1,
    includes = [
        ":message_bridge_server_fbs_includes",
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "message_bridge_server_fbs",
    srcs = ["message_bridge_server.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "team_number",
    srcs = [
        "team_number.cc",
    ],
    hdrs = [
        "team_number.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/util:string_to_num",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "team_number_test",
    srcs = [
        "team_number_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":team_number",
        "//aos:configuration",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "sctp_lib",
    srcs = [
        "sctp_lib.cc",
    ],
    hdrs = [
        "sctp_lib.h",
    ],
    copts = [
        # The casts required to read datastructures from sockets trip -Wcast-align.
        "-Wno-cast-align",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:unique_malloc_ptr",
        "//aos/util:file",
        "//third_party/lksctp-tools:sctp",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "sctp_server",
    srcs = [
        "sctp_server.cc",
    ],
    hdrs = [
        "sctp_server.h",
    ],
    copts = [
        "-Wno-cast-align",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_lib",
        "//third_party/lksctp-tools:sctp",
    ],
)

cc_library(
    name = "message_bridge_protocol",
    srcs = [
        "message_bridge_protocol.cc",
    ],
    hdrs = [
        "message_bridge_protocol.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:uuid",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "message_bridge_server_status",
    srcs = [
        "message_bridge_server_status.cc",
    ],
    hdrs = [
        "message_bridge_server_status.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_client_fbs",
        ":message_bridge_server_fbs",
        ":timestamp_fbs",
        ":timestamp_filter",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/time",
    ],
)

cc_library(
    name = "timestamp_channel",
    srcs = ["timestamp_channel.cc"],
    hdrs = ["timestamp_channel.h"],
    deps = [
        ":remote_message_fbs",
        "//aos:configuration",
        "//aos/events:event_loop",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "message_bridge_server_lib",
    srcs = [
        "message_bridge_server_lib.cc",
    ],
    hdrs = [
        "message_bridge_server_lib.h",
    ],
    copts = [
        "-Wno-cast-align",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        ":message_bridge_client_fbs",
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":message_bridge_server_status",
        ":remote_message_fbs",
        ":sctp_lib",
        ":sctp_server",
        ":timestamp_channel",
        ":timestamp_fbs",
        "//aos:unique_malloc_ptr",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_reader",
        "//third_party/lksctp-tools:sctp",
    ],
)

cc_binary(
    name = "message_bridge_server",
    srcs = [
        "message_bridge_server.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_server_lib",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "sctp_client",
    srcs = [
        "sctp_client.cc",
    ],
    hdrs = [
        "sctp_client.h",
    ],
    copts = [
        "-Wno-cast-align",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_lib",
        "//third_party/lksctp-tools:sctp",
    ],
)

cc_library(
    name = "message_bridge_client_status",
    srcs = [
        "message_bridge_client_status.cc",
    ],
    hdrs = [
        "message_bridge_client_status.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_client_fbs",
        ":message_bridge_server_fbs",
        ":timestamp_filter",
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/time",
    ],
)

cc_library(
    name = "message_bridge_client_lib",
    srcs = [
        "message_bridge_client_lib.cc",
    ],
    hdrs = [
        "message_bridge_client_lib.h",
    ],
    copts = [
        "-Wno-cast-align",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        ":message_bridge_client_fbs",
        ":message_bridge_client_status",
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":remote_message_fbs",
        ":sctp_client",
        ":timestamp_fbs",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_reader",
    ],
)

cc_binary(
    name = "message_bridge_client",
    srcs = [
        "message_bridge_client.cc",
    ],
    copts = [
        "-Wno-cast-align",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_client_lib",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
    ],
)

aos_config(
    name = "message_bridge_test_combined_timestamps_common_config",
    src = "message_bridge_test_combined_timestamps_common.json",
    flatbuffers = [
        ":remote_message_fbs",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:config"],
)

aos_config(
    name = "message_bridge_test_common_config",
    src = "message_bridge_test_common.json",
    flatbuffers = [
        ":remote_message_fbs",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:config"],
)

cc_test(
    name = "message_bridge_test",
    srcs = [
        "message_bridge_test.cc",
    ],
    data = [
        ":message_bridge_test_combined_timestamps_common_config",
        ":message_bridge_test_common_config",
    ],
    flaky = True,
    shard_count = 10,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_client_lib",
        ":message_bridge_server_lib",
        "//aos:json_to_flatbuffer",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/events:shm_event_loop",
        "//aos/ipc_lib:event",
        "//aos/testing:googletest",
    ],
)

flatbuffer_cc_library(
    name = "web_proxy_fbs",
    srcs = ["web_proxy.fbs"],
    gen_reflections = True,
    includes = [
        ":connect_fbs_includes",
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_ts_library(
    name = "web_proxy_ts_fbs",
    srcs = ["web_proxy.fbs"],
    includes = [
        ":connect_fbs_includes",
        "//aos:configuration_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "web_proxy_utils",
    srcs = ["web_proxy_utils.cc"],
    hdrs = ["web_proxy_utils.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        ":web_proxy_fbs",
        "//aos:configuration_fbs",
        "//aos/events:event_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "web_proxy",
    srcs = ["web_proxy.cc"],
    hdrs = ["web_proxy.h"],
    copts = [
        "-DWEBRTC_POSIX",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        ":gen_embedded",
        ":web_proxy_fbs",
        ":web_proxy_utils",
        "//aos/events:shm_event_loop",
        "//aos/mutex",
        "//aos/seasocks:seasocks_logger",
        "//third_party:webrtc",
        "//third_party/seasocks",
        "@com_github_google_glog//:glog",
    ],
)

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "web_proxy_main",
    srcs = ["web_proxy_main.cc"],
    copts = [
        "-DWEBRTC_POSIX",
    ],
    data = [
        "//aos/network/www:files",
        "//aos/network/www:main_bundle.min.js",
        "@com_github_google_flatbuffers//:flatjs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":web_proxy",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_binary(
    name = "log_web_proxy_main",
    srcs = ["log_web_proxy_main.cc"],
    args = [
        "--data_dir=aos/network/www",
    ],
    copts = [
        "-DWEBRTC_POSIX",
    ],
    deps = [
        ":web_proxy",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "timestamp_filter",
    srcs = ["timestamp_filter.cc"],
    hdrs = ["timestamp_filter.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos/time",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "multinode_timestamp_filter",
    srcs = ["multinode_timestamp_filter.cc"],
    hdrs = ["multinode_timestamp_filter.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":timestamp_filter",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:logfile_utils",
        "//aos/time",
        "@com_github_stevengj_nlopt//:nlopt",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "timestamp_filter_test",
    srcs = [
        "timestamp_filter_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":timestamp_filter",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "testing_time_converter",
    testonly = True,
    srcs = ["testing_time_converter.cc"],
    hdrs = ["testing_time_converter.h"],
    deps = [
        ":multinode_timestamp_filter",
        "//aos/events:simulated_event_loop",
        "//aos/time",
    ],
)

cc_test(
    name = "multinode_timestamp_filter_test",
    srcs = [
        "multinode_timestamp_filter_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":multinode_timestamp_filter",
        ":testing_time_converter",
        ":timestamp_filter",
        "//aos/testing:googletest",
        "//third_party/gmp",
        "@com_github_stevengj_nlopt//:nlopt",
    ],
)
