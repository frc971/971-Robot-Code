package(default_visibility = ['//y2017:__subpackages__'])

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
  name = 'indexer',
  srcs = [
    'indexer.py',
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
  ]
)

py_binary(
  name = 'turret',
  srcs = [
    'turret.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_binary(
  name = 'hood',
  srcs = [
    'hood.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_library(
  name = 'turret_lib',
  srcs = [
    'turret.py',
  ],
  deps = [
    '//aos/common/util:py_trapezoid_profile',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_library(
  name = 'indexer_lib',
  srcs = [
    'indexer.py',
  ],
  deps = [
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ]
)

py_binary(
  name = 'column',
  srcs = [
    'column.py',
  ],
  deps = [
    ':turret_lib',
    ':indexer_lib',
    '//external:python-gflags',
    '//external:python-glog',
    '//frc971/control_loops/python:controls',
  ],
)
