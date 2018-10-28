load("//aos/build:queues.bzl", "queue_library")

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
        "//third_party/gflags",
        "//y2018:vision_proto",
    ],
)

queue_library(
    name = "vision_queue",
    srcs = [
        "vision.q",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "vision_status",
    srcs = [
        "vision_status.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_queue",
        "//aos/time:time",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos:init",
        "//aos/vision/events:udp",
        "//y2018:vision_proto",
    ],
)
