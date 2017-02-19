package(default_visibility = ['//visibility:public'])

genrule(
  name = 'genrule_shooter',
  cmd = '$(location //y2017/control_loops/python:shooter) $(OUTS)',
  tools = [
    '//y2017/control_loops/python:shooter',
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
  visibility = ['//visibility:public'],
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
  name = 'shooter',
  visibility = ['//visibility:public'],
  srcs = [
    'shooter.cc',
  ],
  hdrs = [
    'shooter.h',
  ],
  deps = [
    ':shooter_plants',
    '//aos/common/controls:control_loop',
    '//third_party/eigen',
    '//y2017/control_loops/superstructure:superstructure_queue',
  ],
)
