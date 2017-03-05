load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "overlay",
    hdrs = ["overlay.h"],
    deps = [
        '//aos/vision/math:vector',
        '//aos/vision/math:segment',
        '//aos/vision/image:image_types',
        ],
)

gtk_dependent_cc_library(name = "debug_window",
    srcs = ["debug_window.cc"],
    hdrs = ["debug_window.h"],
    deps = [
        '@usr_repo//:gtk+-3.0',
        "//aos/vision/image:image_types",
        ":overlay",
    ]
)

gtk_dependent_cc_library(
  name = 'debug_framework',
  srcs = [
    'debug_framework.cc',
    'jpeg_list-source.cc',
    'tcp-source.cc',
    'blob_log-source.cc',
    'camera-source.cc'
  ],
  hdrs = ['debug_framework.h'],
  deps = [
    '//aos/common/logging:logging',
    '//aos/common/logging:implementations',
    '//aos/vision/blob:codec',
    '//aos/vision/blob:range_image',
    '//aos/vision/blob:stream_view',
    '//aos/vision/blob:find_blob',
    '//aos/vision/events:gtk_event',
    '//aos/vision/events:epoll_events',
    "//aos/vision/events:tcp_client",
    '//aos/vision/image:jpeg_routines',
    '//aos/vision/image:image_stream',
    '//aos/vision/image:image_types',
    '//aos/common/util:global_factory',
    '@usr_repo//:gtk+-3.0',
  ],
  alwayslink = 1,
)
