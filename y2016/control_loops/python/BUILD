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
  name = 'shooter',
  srcs = [
    'shooter.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_binary(
  name = 'intake',
  srcs = [
    'intake.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_binary(
  name = 'shoulder',
  srcs = [
    'shoulder.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_binary(
  name = 'wrist',
  srcs = [
    'wrist.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_library(
  name = 'wrist_lib',
  srcs = [
    'wrist.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_library(
  name = 'shoulder_lib',
  srcs = [
    'shoulder.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)

py_library(
  name = 'arm_lib',
  srcs = [
    'arm.py',
  ],
  deps = [
    ':wrist_lib',
    ':shoulder_lib',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
    '//aos/common/util:py_trapezoid_profile',
  ],
)
py_binary(
  name = 'arm',
  srcs = [
    'arm.py',
  ],
  deps = [
    ':wrist_lib',
    ':shoulder_lib',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
    '//aos/common/util:py_trapezoid_profile',
  ],
)
