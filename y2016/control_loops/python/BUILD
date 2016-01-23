package(default_visibility = ['//y2016:__subpackages__'])

py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
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

py_binary(
  name = 'intake',
  srcs = [
    'intake.py',
  ],
  deps = [
    ':polydrivetrain_lib',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_binary(
  name = 'shoulder',
  srcs = [
    'shoulder.py',
  ],
  deps = [
    ':polydrivetrain_lib',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_binary(
  name = 'wrist',
  srcs = [
    'wrist.py',
  ],
  deps = [
    ':polydrivetrain_lib',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)
