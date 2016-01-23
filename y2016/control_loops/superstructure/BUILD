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
  name = 'genrule_shoulder',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2016/control_loops/python:shoulder) $(OUTS)',
  tools = [
    '//y2016/control_loops/python:shoulder',
  ],
  outs = [
    'shoulder_plant.h',
    'shoulder_plant.cc',
    'integral_shoulder_plant.h',
    'integral_shoulder_plant.cc',
  ],
)

genrule(
  name = 'genrule_wrist',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2016/control_loops/python:wrist) $(OUTS)',
  tools = [
    '//y2016/control_loops/python:wrist',
  ],
  outs = [
    'wrist_plant.h',
    'wrist_plant.cc',
    'integral_wrist_plant.h',
    'integral_wrist_plant.cc',
  ],
)

cc_library(
  name = 'superstructure_plants',
  srcs = [
    'intake_plant.cc',
    'shoulder_plant.cc',
    'wrist_plant.cc',
    'integral_intake_plant.cc',
    'integral_shoulder_plant.cc',
    'integral_wrist_plant.cc',
  ],
  hdrs = [
    'intake_plant.h',
    'shoulder_plant.h',
    'wrist_plant.h',
    'integral_intake_plant.h',
    'integral_shoulder_plant.h',
    'integral_wrist_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
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
    '//frc971/control_loops:state_feedback_loop',
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
