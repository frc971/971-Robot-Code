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
    '//frc971/queues:gyro',
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

genrule(
  name = 'genrule_drivetrain',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2012/control_loops/python:drivetrain) $(OUTS)',
  tools = [
    '//y2012/control_loops/python:drivetrain',
  ],
  outs = [
    'drivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.h',
    'kalman_drivetrain_motor_plant.cc',
  ],
)

genrule(
  name = 'genrule_polydrivetrain',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2012/control_loops/python:polydrivetrain) $(OUTS)',
  tools = [
    '//y2012/control_loops/python:polydrivetrain',
  ],
  outs = [
    'polydrivetrain_dog_motor_plant.h',
    'polydrivetrain_dog_motor_plant.cc',
    'polydrivetrain_cim_plant.h',
    'polydrivetrain_cim_plant.cc',
  ],
)

cc_library(
  name = 'polydrivetrain_plants',
  srcs = [
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.cc',
  ],
  hdrs = [
    'polydrivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.h',
    'kalman_drivetrain_motor_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'ssdrivetrain',
  srcs = [
    'ssdrivetrain.cc',
  ],
  hdrs = [
    'ssdrivetrain.h',
  ],
  deps = [
    ':polydrivetrain_plants',
    ':drivetrain_queue',
    '//aos/common/controls:polytope',
    '//aos/common:math',
    '//aos/common/messages:robot_state',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//aos/common/util:log_interval',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_library(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.cc',
  ],
  hdrs = [
    'polydrivetrain.h',
  ],
  deps = [
    ':polydrivetrain_plants',
    ':drivetrain_queue',
    '//aos/common/controls:polytope',
    '//aos/common:math',
    '//aos/common/messages:robot_state',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//aos/common/util:log_interval',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_library(
  name = 'drivetrain_lib',
  srcs = [
    'drivetrain.cc',
  ],
  hdrs = [
    'drivetrain.h',
  ],
  deps = [
    ':drivetrain_queue',
    ':polydrivetrain',
    ':polydrivetrain_plants',
    ':ssdrivetrain',
    '//aos/common/controls:control_loop',
    '//frc971/queues:gyro',
    '//aos/common/util:log_interval',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
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
