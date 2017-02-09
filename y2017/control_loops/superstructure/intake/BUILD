genrule(
  name = 'genrule_intake',
  cmd = '$(location //y2017/control_loops/python:intake) $(OUTS)',
  tools = [
    '//y2017/control_loops/python:intake',
  ],
  outs = [
    'intake_plant.h',
    'intake_plant.cc',
    'intake_integral_plant.h',
    'intake_integral_plant.cc',
  ],
)

cc_library(
  name = 'intake_plants',
  visibility = ['//visibility:public'],
  srcs = [
    'intake_plant.cc',
    'intake_integral_plant.cc',
  ],
  hdrs = [
    'intake_plant.h',
    'intake_integral_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'intake',
  visibility = ['//visibility:public'],
  srcs = [
    'intake.cc',
  ],
  hdrs = [
    'intake.h',
  ],
  deps = [
    ':intake_plants',
    '//frc971/control_loops:profiled_subsystem',
    '//y2017/control_loops/superstructure:superstructure_queue',
    '//y2017:constants',
  ],
)
