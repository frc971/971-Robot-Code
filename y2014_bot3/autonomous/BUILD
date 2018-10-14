package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

queue_library(
  name = 'auto_queue',
  srcs = [
    'auto.q',
  ],
)

cc_library(
  name = 'auto_lib',
  srcs = [
    'auto.cc',
  ],
  hdrs = [
    'auto.h',
  ],
  deps = [
    ':auto_queue',
    '//aos/controls:control_loop',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2014_bot3/control_loops/rollers:rollers_queue',
    '//aos/time:time',
    '//aos/util:phased_loop',
    '//aos/util:trapezoid_profile',
    '//aos/logging',
    '//aos/logging:queue_logging',
  ],
)

cc_binary(
  name = 'auto',
  srcs = [
    'auto_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':auto_queue',
    ':auto_lib',
  ],
)
