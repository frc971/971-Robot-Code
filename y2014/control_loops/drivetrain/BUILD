package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_binary(
  name = 'replay_drivetrain',
  srcs = [
    'replay_drivetrain.cc',
  ],
  deps = [
    ':drivetrain_queue',
    '//aos/common/controls:replay_control_loop',
    '//aos/linux_code:init',
  ],
)

queue_library(
  name = 'drivetrain_queue',
  srcs = [
    'drivetrain.q',
  ],
)

cc_library(
  name = 'polydrivetrain_plants',
  srcs = [
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'drivetrain_lib',
  srcs = [
    'drivetrain.cc',
    'polydrivetrain_cim_plant.cc',
  ],
  deps = [
    ':drivetrain_queue',
    '//aos/common/controls:control_loop',
    '//y2014:constants',
    '//aos/common/controls:polytope',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//frc971/queues:gyro',
    '//aos/common/util:log_interval',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_binary(
  name = 'drivetrain_lib_test',
  srcs = [
    'drivetrain_lib_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':drivetrain_queue',
    ':drivetrain_lib',
    '//aos/common/controls:control_loop_test',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/queues:gyro',
    '//aos/common:queues',
  ],
)

cc_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':drivetrain_lib',
    ':drivetrain_queue',
  ],
)
