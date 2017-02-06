package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'superstructure_queue',
  srcs = [
    'superstructure.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:profiled_subsystem_queue',
    '//frc971/control_loops:queues',
  ],
)
