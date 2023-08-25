load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary")

gtk_dependent_cc_binary(
    name = "jpeg_vision_test",
    srcs = ["jpeg_vision_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/vision/blob:range_image",
        "//aos/vision/blob:stream_view",
        "//aos/vision/blob:threshold",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:gtk_event",
        "//aos/vision/events:tcp_server",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:jpeg_routines",
        "//aos/vision/image:reader",
        "//aos/vision/math:vector",
    ],
)

cc_binary(
    name = "camera_primer",
    srcs = ["camera_primer.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/vision/events:epoll_events",
        "//aos/vision/image:image_stream",
    ],
)
