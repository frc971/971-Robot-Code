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
    ':superstructure_queue',
    '//aos/common/controls:control_loop',
    '//y2017/control_loops/superstructure/hood',
    '//y2017/control_loops/superstructure/turret',
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
    '//y2017/control_loops/superstructure/hood:hood_plants',
    '//y2017/control_loops/superstructure/intake:intake_plants',
    '//y2017/control_loops/superstructure/shooter:shooter_plants',
    '//y2017/control_loops/superstructure/turret:turret_plants',
  ],
)
