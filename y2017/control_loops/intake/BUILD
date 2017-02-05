package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'intake_queue',
  srcs = [
    'intake.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)
