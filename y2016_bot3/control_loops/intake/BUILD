package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'intake_queue',
  srcs = [
    'intake.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

genrule(
  name = 'genrule_intake',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2016_bot3/control_loops/python:intake) $(OUTS)',
  tools = [
    '//y2016_bot3/control_loops/python:intake',
  ],
  outs = [
    'intake_plant.h',
    'intake_plant.cc',
    'integral_intake_plant.h',
    'integral_intake_plant.cc',
  ],
)

cc_library(
  name = 'intake_plants',
  srcs = [
    'intake_plant.cc',
    'integral_intake_plant.cc',
  ],
  hdrs = [
    'intake_plant.h',
    'integral_intake_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'intake_lib',
  srcs = [
    'intake.cc',
    'intake_controls.cc',
  ],
  hdrs = [
    'intake.h',
    'intake_controls.h',
  ],
  deps = [
    ':intake_queue',
    ':intake_plants',
    '//aos/common/controls:control_loop',
    '//aos/common/util:trapezoid_profile',
    '//aos/common:math',
    '//y2016_bot3/queues:ball_detector',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:simple_capped_state_feedback_loop',
    '//frc971/zeroing',
  ],
)

cc_test(
  name = 'intake_lib_test',
  srcs = [
    'intake_lib_test.cc',
  ],
  deps = [
    ':intake_queue',
    ':intake_lib',
    '//aos/testing:googletest',
    '//aos/common:queues',
    '//aos/common/controls:control_loop_test',
    '//aos/common:math',
    '//aos/common:time',
    '//frc971/control_loops:position_sensor_sim',
  ],
)

cc_binary(
  name = 'intake',
  srcs = [
    'intake_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':intake_lib',
    ':intake_queue',
  ],
)
