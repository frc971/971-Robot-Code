package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'superstructure_queue',
  srcs = [
    'superstructure.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:profiled_subsystem_queue',
    '//frc971/control_loops:queues',
  ],
)

cc_library(
  name = 'superstructure_lib',
  srcs = [
    'superstructure.cc',
  ],
  hdrs = [
    'superstructure.h',
  ],
  deps = [
    ':vision_distance_average',
    ':superstructure_queue',
    '//aos/common/controls:control_loop',
    '//y2017/control_loops/superstructure/column',
    '//y2017/control_loops/superstructure/hood',
    '//y2017/control_loops/superstructure/intake',
    '//y2017/control_loops/superstructure/shooter',
    '//y2017:constants',
  ],
)

cc_test(
  name = 'superstructure_lib_test',
  srcs = [
    'superstructure_lib_test.cc',
  ],
  deps = [
    ':superstructure_queue',
    ':superstructure_lib',
    '//aos/common/controls:control_loop_test',
    '//aos/common:math',
    '//aos/common:queues',
    '//aos/common:time',
    '//aos/testing:googletest',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:team_number_test_environment',
    '//y2017/control_loops/superstructure/column:column_plants',
    '//y2017/control_loops/superstructure/hood:hood_plants',
    '//y2017/control_loops/superstructure/intake:intake_plants',
    '//y2017/control_loops/superstructure/shooter:shooter_plants',
  ],
)

cc_binary(
  name = 'superstructure',
  srcs = [
    'superstructure_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':superstructure_lib',
    ':superstructure_queue',
  ],
)

cc_library(
  name = 'vision_time_adjuster',
  hdrs = [
    'vision_time_adjuster.h',
  ],
  srcs = [
    'vision_time_adjuster.cc',
  ],
  deps = [
    ':superstructure_queue',
    '//aos/common:ring_buffer',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2017/control_loops/drivetrain:polydrivetrain_plants',
    '//y2017/vision:vision_queue',
  ],
)

cc_test(
  name = 'vision_time_adjuster_test',
  srcs = [
    'vision_time_adjuster_test.cc',
  ],
  deps = [
    ':vision_time_adjuster',
    '//aos/common:time',
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
  ],
)

cc_library(
  name = 'vision_distance_average',
  hdrs = [
    'vision_distance_average.h',
  ],
  deps = [
    '//aos/common:time',
    '//aos/common:ring_buffer',
    '//y2017/vision:vision_queue',
  ],
)

cc_test(
  name = 'vision_distance_average_test',
  srcs = [
    'vision_distance_average_test.cc',
  ],
  deps = [
    ':vision_distance_average',
    '//aos/common:time',
    '//aos/testing:googletest',
  ],
)
