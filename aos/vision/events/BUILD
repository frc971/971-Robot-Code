load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "epoll_events",
    srcs = ["epoll_events.cc"],
    hdrs = ["epoll_events.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/common:scoped_fd",
        "//aos/common:time",
        "//aos/common/logging",
    ],
)

cc_library(
    name = "socket_types",
    hdrs = ["socket_types.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/vision/events:tcp_server",
        "//aos/vision/image:image_types",
        "//third_party/protobuf",
    ],
)

cc_library(
    name = "intrusive_free_list",
    hdrs = ["intrusive_free_list.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
)

cc_library(
    name = "tcp_server",
    srcs = ["tcp_server.cc"],
    hdrs = ["tcp_server.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":epoll_events",
        ":intrusive_free_list",
    ],
)

cc_library(
    name = "tcp_client",
    srcs = ["tcp_client.cc"],
    hdrs = ["tcp_client.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [":epoll_events"],
)

cc_library(
    name = "udp",
    srcs = ["udp.cc"],
    hdrs = ["udp.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/common:macros",
        "//aos/common:scoped_fd",
    ],
)

cc_test(
    name = "udp_test",
    srcs = ["udp_test.cc"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":udp",
        "//aos/testing:googletest",
    ],
)

gtk_dependent_cc_library(
    name = "gtk_event",
    srcs = ["gtk_event.cc"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":epoll_events",
        "@usr_repo//:gtk+-3.0",
    ],
)
