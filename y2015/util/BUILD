package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'kinematics',
  deps = [
    '//third_party/eigen',
    '//y2015:constants',
  ],
)

cc_test(
  name = 'kinematics_test',
  srcs = [
    'kinematics_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    '//aos/common:queue_testutils',
    '//aos/common/logging',
    '//frc971/control_loops:team_number_test_environment',
    ':kinematics',
  ],
)
