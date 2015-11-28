package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'elevator_queue',
  srcs = [
    'elevator.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

genrule(
  name = 'genrule_elevator3',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2015_bot3/control_loops/python:elevator3) $(OUTS)',
  tools = [
    '//y2015_bot3/control_loops/python:elevator3',
  ],
  outs = [
    'elevator_motor_plant.h',
    'elevator_motor_plant.cc',
    'integral_elevator_motor_plant.h',
    'integral_elevator_motor_plant.cc',
  ],
)

cc_library(
  name = 'elevator_lib',
  srcs = [
    'elevator.cc',
    'elevator_motor_plant.cc',
    'integral_elevator_motor_plant.cc',
  ],
  hdrs = [
    'elevator.h',
    'elevator_motor_plant.h',
    'integral_elevator_motor_plant.h',
  ],
  deps = [
    ':elevator_queue',
    '//aos/common/logging',
    '//aos/common/controls:control_loop_queues',
    '//aos/common/controls:control_loop',
    '//aos/common/util:trapezoid_profile',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops/voltage_cap:voltage_cap',
  ],
)

cc_test(
  name = 'elevator_lib_test',
  srcs = [
    'elevator_lib_test.cc',
  ],
  deps = [
    '//third_party/googletest',
    ':elevator_lib',
    '//y2015_bot3/control_loops:position_sensor_sim',
    '//frc971/control_loops:state_feedback_loop',
    '//aos/common/controls:control_loop_test',
    '//aos/common:time',
  ],
)

cc_binary(
  name = 'elevator',
  srcs = [
    'elevator_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':elevator_lib',
  ],
)
