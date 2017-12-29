package(default_visibility = ['//visibility:public'])

py_library(
  name = 'controls',
  srcs = [
    'cim.py',
    'control_loop.py',
    'controls.py',
    'polytope.py',
    'libcdd.py',
  ],
  deps = [
    '//external:python-glog',
    '//external:slycot',
  ],
  data = [
    '//third_party/cddlib:_cddlib.so',
  ],
  restricted_to = ['//tools:k8'],
)

py_test(
  name = 'polytope_test',
  srcs = [
    'polytope_test.py',
  ],
  deps = [
    ':controls',
  ],
  restricted_to = ['//tools:k8'],
)

py_binary(
  name = 'down_estimator',
  srcs = [
    'down_estimator.py',
  ],
  deps = [
    ':controls',
  ],
  restricted_to = ['//tools:k8'],
)

py_library(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    ':controls',
  ],
  restricted_to = ['//tools:k8'],
)

py_library(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.py',
  ],
  deps = [
    ':controls',
    ':drivetrain',
    '//external:python-glog',
  ],
  restricted_to = ['//tools:k8'],
)
