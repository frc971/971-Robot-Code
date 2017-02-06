package(default_visibility = ['//visibility:public'])

genrule(
  name = 'genrule_hood',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2017/control_loops/python:hood) $(OUTS)',
  tools = [
    '//y2017/control_loops/python:hood',
  ],
  outs = [
    'hood_plant.h',
    'hood_plant.cc',
    'hood_integral_plant.h',
    'hood_integral_plant.cc',
  ],
)

cc_library(
  name = 'hood_plants',
  srcs = [
    'hood_plant.cc',
    'hood_integral_plant.cc',
  ],
  hdrs = [
    'hood_plant.h',
    'hood_integral_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'hood',
  srcs = [
    'hood.cc',
  ],
  hdrs = [
    'hood.h',
  ],
  deps = [
    ':hood_plants',
    '//frc971/control_loops:profiled_subsystem',
    '//y2017/control_loops/superstructure:superstructure_queue',
    '//y2017:constants',
  ],
)
