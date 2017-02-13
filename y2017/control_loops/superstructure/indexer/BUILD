genrule(
  name = 'genrule_indexer',
  cmd = '$(location //y2017/control_loops/python:indexer) $(OUTS)',
  tools = [
    '//y2017/control_loops/python:indexer',
  ],
  outs = [
    'indexer_plant.h',
    'indexer_plant.cc',
    'indexer_integral_plant.h',
    'indexer_integral_plant.cc',
  ],
)

cc_library(
  name = 'indexer_plants',
  visibility = ['//visibility:public'],
  srcs = [
    'indexer_plant.cc',
    'indexer_integral_plant.cc',
  ],
  hdrs = [
    'indexer_plant.h',
    'indexer_integral_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)
