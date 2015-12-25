package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'kinematics',
  hdrs = [
    'kinematics.h',
  ],
  deps = [
    '//third_party/eigen',
    '//y2015:constants',
    '//debian:libm',
  ],
)

cc_test(
  name = 'kinematics_test',
  srcs = [
    'kinematics_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/testing:test_logging',
    '//aos/common/logging',
    '//frc971/control_loops:team_number_test_environment',
    ':kinematics',
  ],
)
