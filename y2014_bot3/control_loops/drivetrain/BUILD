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
  deps = [
    '//aos/common/controls:control_loop_queues',
  ],
)

cc_library(
  name = 'polydrivetrain_plants',
  srcs = [
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
    'polydrivetrain_cim_plant.cc',
  ],
  hdrs = [
    'polydrivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.h',
    'polydrivetrain_cim_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'drivetrain_lib',
  hdrs = [
    'drivetrain.h',
  ],
  srcs = [
    'drivetrain.cc',
  ],
  deps = [
    ':polydrivetrain_plants',
    ':drivetrain_queue',
    '//aos/common/controls:control_loop',
    '//aos/common/controls:polytope',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//frc971/queues:gyro',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
    '//aos/common:math',
    '//aos/common/util:log_interval',
    '//frc971:shifter_hall_effect',
  ],
)

cc_test(
  name = 'drivetrain_lib_test',
  srcs = [
    'drivetrain_lib_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':drivetrain_queue',
    ':drivetrain_lib',
    '//aos/common/controls:control_loop_test',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/queues:gyro',
    '//aos/common:queues',
    '//aos/common/network:team_number',
  ],
)

cc_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain_main.cc',
  ],
  deps = [
    ':drivetrain_lib',
    ':drivetrain_queue',
    '//aos/linux_code:init',
  ],
)
