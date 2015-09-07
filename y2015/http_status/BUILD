package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'http_status',
  srcs = [
    'http_status.cc',
  ],
  hdrs = [
    'http_status.h',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//third_party/seasocks',
    '//y2015/control_loops/claw:claw_queue',
    '//y2015/control_loops/fridge:fridge_queue',
    '//aos/common/util:phased_loop',
    '//aos/common:time',
  ],
)
