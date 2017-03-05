load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')

gtk_dependent_cc_binary(name = "jpeg_vision_test",
  srcs = ["jpeg_vision_test.cc"],
  deps = [
    "//aos/common/logging:logging",
    "//aos/common/logging:implementations",
    "//aos/vision/math:vector",
    "//aos/vision/image:reader",
    "//aos/vision/image:jpeg_routines",
    "//aos/vision/blob:threshold",
    "//aos/vision/blob:range_image",
    "//aos/vision/events:epoll_events",
    "//aos/vision/blob:stream_view",
    "//aos/vision/image:image_stream",
    "//aos/vision/events:tcp_server",
    "//aos/vision/events:gtk_event",
  ],
)

cc_binary(
  name = 'camera_primer',
  srcs = ['camera_primer.cc'],
  deps = [
    '//aos/common/logging:logging',
    '//aos/common/logging:implementations',
    '//aos/vision/image:image_stream',
    '//aos/vision/events:epoll_events',
  ],
)
