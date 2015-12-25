licenses(['notice'])

cc_library(
  name = 'ntcore',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/**/*.cpp',
    'src/**/*.h',
  ]),
  copts = [
    '-Wno-switch-enum',
    '-Wno-cast-align',
  ],
  hdrs = glob([
    'include/**/*.h',
  ]),
  includes = [
    'include',
  ],
  linkopts = [
    '-lpthread',
  ],
)
