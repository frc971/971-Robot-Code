package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "image_streamer",
    srcs = ["image_streamer.cc"],
    deps = [
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/vision/blob:codec",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:reader",
        "//y2019:vision_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)
