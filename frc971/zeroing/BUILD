package(default_visibility = ['//visibility:public'])

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
  ],
)

cc_test(
  name = 'zeroing_test',
  srcs = [
    'zeroing_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    '//aos/common:queue_testutils',
    ':zeroing',
    '//aos/common/util:thread',
    '//aos/common:die',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:queues',
  ],
)
