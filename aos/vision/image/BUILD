package(default_visibility = ['//visibility:public'])
load('/tools/build_rules/protobuf', 'proto_cc_library')

cc_library(
  name = 'image_types',
  hdrs = ['image_types.h'],
  deps = [
    '//aos/common/logging:logging',
  ],
)

proto_cc_library(
  name = 'camera_params',
  src = 'camera_params.proto',
)

cc_library(
  name = 'reader',
  srcs = ['reader.cc'],
  hdrs = ['V4L2.h', 'reader.h'],
  deps = [
    '//aos/common:time',
    '//aos/common/logging:logging',
    ':image_types',
    ':camera_params',
  ],
)

cc_library(
  name = 'jpeg_routines',
  srcs = ['jpeg_routines.cc'],
  hdrs = ['jpeg_routines.h'],
  deps = [
    '//third_party/libjpeg',
    '//aos/common/logging:logging',
    ':image_types'
  ],
)

cc_library(name = 'image_stream',
  hdrs = ['image_stream.h'],
  deps = [
    '//aos/vision/events:epoll_events',
    '//aos/vision/image:reader'
  ]
)
