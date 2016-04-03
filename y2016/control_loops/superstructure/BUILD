package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'superstructure_queue',
  srcs = [
    'superstructure.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

genrule(
  name = 'genrule_intake',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2016/control_loops/python:intake) $(OUTS)',
  tools = [
    '//y2016/control_loops/python:intake',
  ],
  outs = [
    'intake_plant.h',
    'intake_plant.cc',
    'integral_intake_plant.h',
    'integral_intake_plant.cc',
  ],
)

genrule(
  name = 'genrule_arm',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2016/control_loops/python:arm) $(OUTS)',
  tools = [
    '//y2016/control_loops/python:arm',
  ],
  outs = [
    'arm_plant.h',
    'arm_plant.cc',
    'integral_arm_plant.h',
    'integral_arm_plant.cc',
  ],
)

cc_library(
  name = 'superstructure_plants',
  srcs = [
    'intake_plant.cc',
    'arm_plant.cc',
    'integral_intake_plant.cc',
    'integral_arm_plant.cc',
  ],
  hdrs = [
    'intake_plant.h',
    'arm_plant.h',
    'integral_intake_plant.h',
    'integral_arm_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'superstructure_lib',
  srcs = [
    'superstructure.cc',
    'superstructure_controls.cc',
  ],
  hdrs = [
    'superstructure.h',
    'superstructure_controls.h',
  ],
  deps = [
    ':superstructure_queue',
    ':superstructure_plants',
    '//aos/common/controls:control_loop',
    '//aos/common/util:trapezoid_profile',
    '//aos/common:math',
    '//y2016/queues:ball_detector',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:simple_capped_state_feedback_loop',
    '//frc971/zeroing',
    '//y2016:constants',
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
    '//aos/testing:googletest',
    '//aos/common:queues',
    '//aos/common/controls:control_loop_test',
    '//aos/common:math',
    '//aos/common:time',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:team_number_test_environment',
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
