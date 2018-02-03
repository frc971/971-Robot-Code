package(default_visibility = ['//y2018:__subpackages__'])

py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:drivetrain',
  ],
  restricted_to = ['//tools:k8'],
)

py_binary(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:polydrivetrain',
  ],
  restricted_to = ['//tools:k8'],
)

py_library(
  name = 'polydrivetrain_lib',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
    '//frc971/control_loops/python:drivetrain',
  ],
  restricted_to = ['//tools:k8'],
)

py_test(
  name = 'polydrivetrain_test',
  srcs = [
    'polydrivetrain_test.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:polydrivetrain',
    '//frc971/control_loops/python:controls',
    ':polydrivetrain_lib',
  ],
  restricted_to = ['//tools:k8'],
)

py_binary(
  name = 'intake',
  srcs = [
    'intake.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
  restricted_to = ['//tools:k8'],
)
