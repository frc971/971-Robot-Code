package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'zeroing_queue',
  srcs = [
    'zeroing_queue.q',
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
