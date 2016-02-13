package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'shooter_queue',
  srcs = [
    'shooter.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

genrule(
  name = 'genrule_shooter',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2016/control_loops/python:shooter) $(OUTS)',
  tools = [
    '//y2016/control_loops/python:shooter',
  ],
  outs = [
    'shooter_plant.h',
    'shooter_plant.cc',
    'shooter_integral_plant.h',
    'shooter_integral_plant.cc',
  ],
)

cc_library(
  name = 'shooter_plants',
  srcs = [
    'shooter_plant.cc',
    'shooter_integral_plant.cc',
  ],
  hdrs = [
    'shooter_plant.h',
    'shooter_integral_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'shooter_lib',
  srcs = [
    'shooter.cc',
  ],
  hdrs = [
    'shooter.h',
  ],
  deps = [
    ':shooter_queue',
    ':shooter_plants',
    '//aos/common/controls:control_loop',
  ],
)

cc_test(
  name = 'shooter_lib_test',
  srcs = [
    'shooter_lib_test.cc',
  ],
  deps = [
    ':shooter_queue',
    ':shooter_lib',
    '//aos/testing:googletest',
    '//aos/common:queues',
    '//aos/common/controls:control_loop_test',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:team_number_test_environment',
  ],
)

cc_binary(
  name = 'shooter',
  srcs = [
    'shooter_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':shooter_lib',
    ':shooter_queue',
  ],
)
