package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

cc_binary(
  name = 'accessories',
  srcs = [
    'accessories.cc',
  ],
  deps = [
    ':accessories_queue',
    '//aos/linux_code:init',
    '//aos/controls:control_loop',
  ],
)

queue_library(
  name = 'accessories_queue',
  srcs = [
    'accessories.q',
  ],
  deps = [
    '//aos/controls:control_loop_queues',
  ],
)
