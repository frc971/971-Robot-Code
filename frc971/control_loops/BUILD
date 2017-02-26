package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_library(
  name = 'team_number_test_environment',
  srcs = [
    'team_number_test_environment.cc',
  ],
  hdrs = [
    'team_number_test_environment.h',
  ],
  deps = [
    '//aos/common/network:team_number',
    '//aos/testing:googletest',
  ],
  testonly = True,
)

cc_test(
  name = 'state_feedback_loop_test',
  srcs = [
    'state_feedback_loop_test.cc',
  ],
  deps = [
    ':state_feedback_loop',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'hall_effect_tracker',
  hdrs = [
    'hall_effect_tracker.h',
  ],
  deps = [
    ':queues',
  ],
)

queue_library(
  name = 'queues',
  srcs = [
    'control_loops.q',
  ],
)

cc_test(
  name = 'position_sensor_sim_test',
  srcs = [
    'position_sensor_sim_test.cc',
  ],
  deps = [
    ':queues',
    ':position_sensor_sim',
    '//aos/testing:googletest',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'position_sensor_sim',
  testonly = True,
  srcs = [
    'position_sensor_sim.cc',
  ],
  hdrs = [
    'position_sensor_sim.h',
  ],
  deps = [
    ':queues',
    ':gaussian_noise',
    '//debian:libm',
    '//aos/testing:random_seed',
  ],
)

cc_library(
  name = 'gaussian_noise',
  srcs = [
    'gaussian_noise.cc',
  ],
  hdrs = [
    'gaussian_noise.h',
  ],
  deps = [
    '//debian:libm',
  ],
)

cc_library(
  name = 'coerce_goal',
  srcs = [
    'coerce_goal.cc',
  ],
  hdrs = [
    'coerce_goal.h',
  ],
  deps = [
    '//third_party/eigen',
    '//aos/common/controls:polytope',
    '//debian:libm',
  ],
)

cc_library(
  name = 'state_feedback_loop',
  hdrs = [
    'state_feedback_loop.h',
  ],
  deps = [
    '//aos/common/controls:control_loop',
    '//aos/common/logging',
    '//aos/common:macros',
    '//third_party/eigen',
  ],
)

cc_library(
  name = 'simple_capped_state_feedback_loop',
  hdrs = [
    'simple_capped_state_feedback_loop.h',
  ],
  deps = [
    '//third_party/eigen',
    ':state_feedback_loop',
  ],
)

cc_library(
  name = 'runge_kutta',
  hdrs = [
    'runge_kutta.h',
  ],
  deps = [
    '//third_party/eigen',
  ],
)

cc_test(
  name = 'runge_kutta_test',
  srcs = [
    'runge_kutta_test.cc',
  ],
  deps = [
    ':runge_kutta',
    '//aos/testing:googletest',
    '//third_party/eigen',
  ],
)

queue_library(
  name = 'profiled_subsystem_queue',
  srcs = [
    'profiled_subsystem.q',
  ],
  deps = [
    ':queues',
  ],
)

cc_library(
  name = 'profiled_subsystem',
  srcs = [
    'profiled_subsystem.cc',
  ],
  hdrs = [
    'profiled_subsystem.h',
  ],
  deps = [
    ':profiled_subsystem_queue',
    ':simple_capped_state_feedback_loop',
    ':state_feedback_loop',
    '//aos/common/controls:control_loop',
    '//aos/common/util:trapezoid_profile',
    '//frc971/zeroing:zeroing',
  ],
)
