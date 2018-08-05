package(default_visibility = ['@//debian:__pkg__'])

cc_library(
  name = 'python3.4_lib',
  hdrs = glob(['usr/include/python3.4m/**/*.h']),
  includes = [
    'usr/include/python3.4m/',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python3.4_f2py',
  srcs = [
    'usr/lib/python3/dist-packages/numpy/f2py/src/fortranobject.c',
  ],
  hdrs = [
    'usr/lib/python3/dist-packages/numpy/f2py/src/fortranobject.h',
  ],
  copts = [
    '-Wno-error',
    '-Wno-parentheses-equality',
  ],
  includes = [
    'usr/lib/python3/dist-packages/numpy/f2py/src/',
  ],
  deps = [
    ':python3.4_lib',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python2.7_lib',
  hdrs = glob([
    'usr/include/**/*.h',
  ]),
  srcs = [
    'usr/lib/x86_64-linux-gnu/libpython2.7.so',
  ],
  includes = [
    'usr/include/',
    'usr/include/python2.7/',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python2.7_f2py',
  srcs = [
    'usr/lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.c',
  ],
  hdrs = [
    'usr/lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.h',
  ],
  copts = [
    '-Wno-error',
  ],
  includes = [
    'usr/lib/python2.7/dist-packages/numpy/f2py/src/',
  ],
  deps = [
    ':python2.7_lib',
  ],
  visibility = ['//visibility:public'],
)
