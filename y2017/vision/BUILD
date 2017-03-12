load('/aos/build/queues', 'queue_library')
load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')
load('/tools/build_rules/protobuf', 'proto_cc_library')

package(default_visibility = ["//visibility:public"])

queue_library(
  name = 'vision_queue',
  visibility = ['//visibility:public'],
  srcs = [
    'vision.q',
  ],
)

proto_cc_library(
  name = 'vision_result',
  src = 'vision_result.proto',
)

proto_cc_library(
  name = 'vision_config',
  src = 'vision_config.proto',
  deps = [
    '//aos/vision/image:camera_params',
  ],
)

cc_binary(
  name = 'target_sender',
  srcs = [
    'target_sender.cc',
  ],
  deps = [
    ':vision_result',
    ':vision_config',
    ':target_finder',
    '//aos/common/logging:logging',
    '//aos/common/logging:implementations',
    '//aos/common:time',
    '//aos/vision/image:reader',
    '//aos/vision/image:jpeg_routines',
    '//aos/vision/image:image_stream',
    '//aos/vision/events:udp',
    '//aos/vision/events:epoll_events',
    '//aos/vision/events:socket_types',
    '//aos/vision/blob:range_image',
    '//aos/vision/blob:codec',
    '//aos/vision/blob:find_blob',
    '//aos/vision/blob:threshold',
  ],
)

cc_binary(
  name = 'target_receiver',
  srcs = [
    'target_receiver.cc',
  ],
  visibility = ['//visibility:public'],
  deps = [
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/linux_code:init',
    '//aos/common:time',
    '//aos/vision/events:udp',
    ':vision_queue',
    ':vision_result',
    ':target_finder',
    '//aos/common:mutex',
  ],
)

cc_library(
  name = 'target_finder',
  srcs = ['target_finder.cc'],
  hdrs = ['target_finder.h'],
  deps = [
    '//aos/vision/blob:threshold',
    '//aos/vision/blob:transpose',
    '//aos/vision/debug:overlay',
    '//aos/vision/math:vector',
  ],
)

gtk_dependent_cc_binary(
  name = 'debug_viewer',
  srcs = ['debug_viewer.cc'],
  deps = [
    ':target_finder',
    '//aos/vision/blob:threshold',
    '//aos/vision/blob:transpose',
    '//aos/vision/blob:move_scale',
    '//aos/vision/math:vector',
    '//aos/vision/debug:debug_framework',
  ]
)
