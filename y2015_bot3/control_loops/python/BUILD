package(default_visibility = ['//y2015_bot3:__subpackages__'])

py_binary(
  name = 'elevator3',
  srcs = [
    'elevator3.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
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
    '//frc971/control_loops/python:controls',
  ],
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
  ],
)
