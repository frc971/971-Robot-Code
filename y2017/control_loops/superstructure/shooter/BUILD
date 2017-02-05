package(default_visibility = ['//visibility:public'])

genrule(
  name = 'genrule_shooter',
  visibility = ['//visibility:private'],
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
