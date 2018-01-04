load("//tools:environments.bzl", "mcu_cpus")

filegroup(
  name = 'linkerscript',
  visibility = ['//tools/cpp:__pkg__'],
  srcs = [ 'mk64fx512.ld' ],
)

cc_library(
  name = 'core',
  visibility = ['//visibility:public'],
  hdrs = [
    'kinetis.h',
    'nonstd.h',
    'time.h',
  ],
  srcs = [
    'mk20dx128.c',
    'nonstd.c',
    'time.cc',
  ],
  restricted_to = mcu_cpus,
)
