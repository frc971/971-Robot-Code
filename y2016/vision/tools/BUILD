load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")

gtk_dependent_cc_binary(
    name = "blob_stream_replay",
    srcs = ["blob_stream_replay.cc"],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/vision/blob:range_image",
        "//aos/vision/blob:stream_view",
        "//aos/vision/debug:debug_window",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:gtk_event",
        "//aos/vision/events:tcp_server",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:jpeg_routines",
        "//aos/vision/image:reader",
        "//y2016/vision:blob_filters",
    ],
)
