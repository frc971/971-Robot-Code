package(default_visibility = ['//y2014:__subpackages__'])

py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    '//frc971/control_loops/python:controls',
  ],
)

py_library(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//frc971/control_loops/python:controls',
  ],
)

py_binary(
  name = 'claw',
  srcs = [
    'claw.py',
  ],
  deps = [
    ':polydrivetrain',
    '//frc971/control_loops/python:controls',
  ]
)
