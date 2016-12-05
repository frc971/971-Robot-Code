package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_binary(
  name = 'replay_fridge',
  srcs = [
    'replay_fridge.cc',
  ],
  deps = [
    ':fridge_queue',
    '//aos/common/controls:replay_control_loop',
    '//aos/linux_code:init',
  ],
)

queue_library(
  name = 'fridge_queue',
  srcs = [
    'fridge.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
    '//frc971/zeroing:zeroing_queue',
  ],
)

genrule(
  name = 'genrule_elevator',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2015/control_loops/python:elevator) $(OUTS)',
  tools = [
    '//y2015/control_loops/python:elevator',
  ],
  outs = [
    'elevator_motor_plant.h',
    'elevator_motor_plant.cc',
  ],
)

cc_library(
  name = 'elevator_plants',
  srcs = [
    'elevator_motor_plant.cc',
  ],
  hdrs = [
    'elevator_motor_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'fridge_lib',
  srcs = [
    'fridge.cc',
    'integral_arm_plant.cc',
  ],
  hdrs = [
    'fridge.h',
    'integral_arm_plant.h',
  ],
  deps = [
    ':elevator_plants',
    ':fridge_queue',
    '//aos/common/controls:control_loop',
    '//aos/common/util:trapezoid_profile',
    '//frc971/control_loops/voltage_cap:voltage_cap',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/zeroing',
    '//y2015/util:kinematics',
    '//y2015:constants',
  ],
)

cc_test(
  name = 'fridge_lib_test',
  srcs = [
    'fridge_lib_test.cc',
    'arm_motor_plant.cc',
    'arm_motor_plant.h',
  ],
  deps = [
    '//aos/testing:googletest',
    ':fridge_lib',
    '//frc971/control_loops:state_feedback_loop',
    '//aos/common/controls:control_loop_test',
    '//aos/common:time',
    '//aos/common:math',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:team_number_test_environment',
    '//y2015/util:kinematics',
  ],
)

cc_binary(
  name = 'fridge',
  srcs = [
    'fridge_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':fridge_lib',
  ],
)
