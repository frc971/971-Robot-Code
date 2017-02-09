genrule(
  name = 'genrule_turret',
  cmd = '$(location //y2017/control_loops/python:turret) $(OUTS)',
  tools = [
    '//y2017/control_loops/python:turret',
  ],
  outs = [
    'turret_plant.h',
    'turret_plant.cc',
    'turret_integral_plant.h',
    'turret_integral_plant.cc',
  ],
)

cc_library(
  name = 'turret_plants',
  visibility = ['//visibility:public'],
  srcs = [
    'turret_plant.cc',
    'turret_integral_plant.cc',
  ],
  hdrs = [
    'turret_plant.h',
    'turret_integral_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'turret',
  visibility = ['//visibility:public'],
  srcs = [
    'turret.cc',
  ],
  hdrs = [
    'turret.h',
  ],
  deps = [
    ':turret_plants',
    '//frc971/control_loops:profiled_subsystem',
    '//y2017/control_loops/superstructure:superstructure_queue',
    '//y2017:constants',
  ],
)
