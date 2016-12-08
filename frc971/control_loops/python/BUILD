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
)

py_test(
  name = 'polytope_test',
  srcs = [
    'polytope_test.py',
  ],
  deps = [
    ':controls',
  ],
)

py_binary(
  name = 'down_estimator',
  srcs = [
    'down_estimator.py',
  ],
  deps = [
    ':controls',
  ],
)
