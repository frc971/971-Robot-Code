package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_library(
  name = 'averager',
  hdrs = [
    'averager.h',
  ],
)

cc_test(
  name = 'averager_test',
  srcs = [
    'averager_test.cc',
  ],
  deps = [
    ':averager',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'zeroing',
  srcs = [
    'zeroing.cc',
  ],
  hdrs = [
    'zeroing.h',
  ],
  deps = [
    ':wrap',
    '//frc971/control_loops:queues',
    '//frc971:constants',
  ],
)

cc_test(
  name = 'zeroing_test',
  srcs = [
    'zeroing_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
    ':zeroing',
    '//aos/common/util:thread',
    '//aos/common:die',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:queues',
  ],
)

cc_library(
  name = 'wrap',
  srcs = [
    'wrap.cc',
  ],
  hdrs = [
    'wrap.h',
  ],
)

cc_test(
  name = 'wrap_test',
  srcs = [
    'wrap_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':wrap',
  ],
)
