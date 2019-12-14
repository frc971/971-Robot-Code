load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
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
        "//aos/logging",
        "//aos/util:string_to_num",
        "@com_google_absl//absl/base",
    ],
)

cc_test(
    name = "team_number_test",
    srcs = [
        "team_number_test.cc",
    ],
    deps = [
        ":team_number",
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
    hdrs = [
        "message_bridge_protocol.h",
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
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":sctp_lib",
        ":sctp_server",
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
