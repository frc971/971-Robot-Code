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

cc_library(
  name = 'drivetrain_config',
  hdrs = [
    'drivetrain_config.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
    '//frc971:shifter_hall_effect',
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
    ':drivetrain_queue',
    ':drivetrain_config',
    '//aos/common/controls:polytope',
    '//aos/common:math',
    '//aos/common/messages:robot_state',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//frc971:shifter_hall_effect',
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
    ':drivetrain_queue',
    ':drivetrain_config',
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
    ':ssdrivetrain',
    '//aos/common/controls:control_loop',
    '//frc971/queues:gyro',
    '//aos/common/util:log_interval',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
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
    ':drivetrain_config',
    '//aos/common/controls:control_loop_test',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/queues:gyro',
    '//aos/common:queues',
    '//y2016:constants',
    '//y2016/control_loops/drivetrain:polydrivetrain_plants',
  ],
)
