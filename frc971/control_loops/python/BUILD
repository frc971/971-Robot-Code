package(default_visibility = ['//visibility:public'])

py_library(
  name = 'controls',
  srcs = [
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
