load('/aos/externals/seasocks/gen_embedded', 'gen_embedded')

gen_embedded(
  name = 'gen_embedded',
  srcs = glob(['www_defaults/**/*']),
)

cc_binary(
  name = 'http_status',
  visibility = ['//visibility:public'],
  srcs = [
    'http_status.cc',
    'http_status.h',
  ],
  deps = [
    ':gen_embedded',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//third_party/seasocks',
    '//y2015/control_loops/claw:claw_queue',
    '//y2015/control_loops/fridge:fridge_queue',
    '//aos/common/util:phased_loop',
    '//aos/common:time',
  ],
)
