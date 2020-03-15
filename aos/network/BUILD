load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "connect_fbs",
    srcs = ["connect.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
)

flatbuffer_ts_library(
    name = "connect_ts_fbs",
    srcs = ["connect.fbs"],
    includes = [
        "//aos:configuration_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "timestamp_fbs",
    srcs = ["timestamp.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "message_bridge_client_fbs",
    srcs = ["message_bridge_client.fbs"],
    gen_reflections = 1,
    includes = [
        ":message_bridge_server_fbs_includes",
        "//aos:configuration_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "message_bridge_server_fbs",
    srcs = ["message_bridge_server.fbs"],
    gen_reflections = 1,
    includes = [
        "//aos:configuration_fbs_includes",
    ],
)

cc_library(
    name = "team_number",
    srcs = [
        "team_number.cc",
    ],
    hdrs = [
        "team_number.h",
    ],
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
    deps = [
        ":connect_fbs",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "@com_github_google_flatbuffers//:flatbuffers",
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
    deps = [
        ":connect_fbs",
        ":message_bridge_client_fbs",
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":sctp_lib",
        ":sctp_server",
        ":timestamp_fbs",
        "//aos:unique_malloc_ptr",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:logger",
        "//third_party/lksctp-tools:sctp",
    ],
)

cc_binary(
    name = "message_bridge_server",
    srcs = [
        "message_bridge_server.cc",
    ],
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
    deps = [
        ":sctp_lib",
        "//third_party/lksctp-tools:sctp",
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
    deps = [
        ":connect_fbs",
        ":message_bridge_client_fbs",
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":sctp_client",
        ":timestamp_fbs",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:logger",
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
    deps = [
        ":message_bridge_client_lib",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
    ],
)

aos_config(
    name = "message_bridge_test_common_config",
    src = "message_bridge_test_common.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:config"],
)

aos_config(
    name = "message_bridge_test_server_config",
    src = "message_bridge_test_server.json",
    deps = [":message_bridge_test_common_config"],
)

aos_config(
    name = "message_bridge_test_client_config",
    src = "message_bridge_test_client.json",
    deps = [":message_bridge_test_common_config"],
)

cc_test(
    name = "message_bridge_test",
    srcs = [
        "message_bridge_test.cc",
    ],
    data = [
        ":message_bridge_test_client_config.json",
        ":message_bridge_test_server_config.json",
    ],
    deps = [
        ":message_bridge_client_lib",
        ":message_bridge_server_lib",
        "//aos:json_to_flatbuffer",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/events:shm_event_loop",
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
)

flatbuffer_ts_library(
    name = "web_proxy_ts_fbs",
    srcs = ["web_proxy.fbs"],
    includes = [
        ":connect_fbs_includes",
        "//aos:configuration_fbs_includes",
    ],
)

cc_library(
    name = "web_proxy_utils",
    srcs = ["web_proxy_utils.cc"],
    hdrs = ["web_proxy_utils.h"],
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
        "-Wno-unused-parameter",
    ],
    deps = [
        ":connect_fbs",
        ":web_proxy_fbs",
        ":web_proxy_utils",
        "//aos/events:shm_event_loop",
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
)

cc_binary(
    name = "web_proxy_main",
    srcs = ["web_proxy_main.cc"],
    copts = [
        "-DWEBRTC_POSIX",
        "-Wno-unused-parameter",
    ],
    data = [
        "//aos/events:pingpong_config.json",
        "//aos/network/www:files",
        "//aos/network/www:flatbuffers",
        "//aos/network/www:main_bundle",
        "@com_github_google_flatbuffers//:flatjs",
    ],
    deps = [
        ":gen_embedded",
        ":web_proxy",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/seasocks:seasocks_logger",
        "//third_party/seasocks",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "timestamp_filter",
    hdrs = ["timestamp_filter.h"],
    deps = [
        "//aos/time",
    ],
)

cc_test(
    name = "timestamp_filter_test",
    srcs = [
        "timestamp_filter_test.cc",
    ],
    deps = [
        ":timestamp_filter",
        "//aos/testing:googletest",
    ],
)
