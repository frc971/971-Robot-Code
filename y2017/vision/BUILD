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
  name = 'vision_data',
  src = 'vision_data.proto',
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
    ':vision_data',
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
)
