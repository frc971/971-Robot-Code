load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'turret_queue',
  srcs = [
    'turret.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
  visibility = ['//visibility:public'],
)
