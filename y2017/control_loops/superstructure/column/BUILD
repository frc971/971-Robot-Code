genrule(
  name = 'genrule_column',
  cmd = '$(location //y2017/control_loops/python:column) $(OUTS)',
  tools = [
    '//y2017/control_loops/python:column',
  ],
  outs = [
    'column_plant.h',
    'column_plant.cc',
    'column_integral_plant.h',
    'column_integral_plant.cc',
    'stuck_column_integral_plant.h',
    'stuck_column_integral_plant.cc',
  ],
)

cc_library(
  name = 'column_plants',
  visibility = ['//visibility:public'],
  srcs = [
    'column_plant.cc',
    'column_integral_plant.cc',
    'stuck_column_integral_plant.cc',
  ],
  hdrs = [
    'column_plant.h',
    'column_integral_plant.h',
    'stuck_column_integral_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'column',
  visibility = ['//visibility:public'],
  srcs = [
    'column.cc',
  ],
  hdrs = [
    'column.h',
  ],
  deps = [
    ':column_plants',
    ':column_zeroing',
    '//aos/common/controls:control_loop',
    '//aos/common:math',
    '//frc971/control_loops:profiled_subsystem',
    '//y2017/control_loops/superstructure/intake:intake',
    '//y2017/control_loops/superstructure:superstructure_queue',
    '//y2017/control_loops/superstructure:vision_time_adjuster',
    '//y2017:constants',
  ],
)

cc_library(
  name = 'column_zeroing',
  srcs = [
    'column_zeroing.cc',
  ],
  hdrs = [
    'column_zeroing.h',
  ],
  deps = [
    '//frc971/control_loops:queues',
    '//frc971/zeroing:wrap',
    '//frc971/zeroing:zeroing',
    '//frc971:constants',
    '//y2017/control_loops/superstructure:superstructure_queue',
    '//y2017:constants',
  ],
)

cc_test(
  name = 'column_zeroing_test',
  srcs = [
    'column_zeroing_test.cc',
  ],
  deps = [
    ':column_zeroing',
    '//aos/testing:test_shm',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:team_number_test_environment',
    '//y2017/control_loops/superstructure:superstructure_queue',
    '//y2017:constants',
  ],
)
