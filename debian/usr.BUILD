package(default_visibility = ['//debian:__pkg__'])

cc_library(
  name = 'librt',
  srcs = [
    'lib/x86_64-linux-gnu/librt.so'
  ],
)

cc_library(
  name = 'libdl',
  srcs = [
    'lib/x86_64-linux-gnu/libdl.so'
  ],
)

cc_library(
  name = 'libm',
  srcs = [
    'lib/x86_64-linux-gnu/libm.so'
  ],
)

cc_library(
  name = 'libpthread',
)

cc_library(
  name = 'python3.4_lib',
  hdrs = glob(['include/python3.4m/**/*.h']),
  includes = [
    'include/python3.4m/',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python3.4_f2py',
  srcs = [
    'lib/python3/dist-packages/numpy/f2py/src/fortranobject.c',
  ],
  hdrs = [
    'lib/python3/dist-packages/numpy/f2py/src/fortranobject.h',
  ],
  copts = [
    '-Wno-error',
    '-Wno-parentheses-equality',
  ],
  includes = [
    'lib/python3/dist-packages/numpy/f2py/src/',
  ],
  deps = [
    ':python3.4_lib',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python2.7_lib',
  hdrs = glob(['include/python2.7/**/*.h']),
  srcs = [
    'lib/x86_64-linux-gnu/libpython2.7.so',
  ],
  includes = [
    'include/python2.7/',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python2.7_f2py',
  srcs = [
    'lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.c',
  ],
  hdrs = [
    'lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.h',
  ],
  copts = [
    '-Wno-error',
  ],
  includes = [
    'lib/python2.7/dist-packages/numpy/f2py/src/',
  ],
  deps = [
    ':python2.7_lib',
  ],
  visibility = ['//visibility:public'],
)
