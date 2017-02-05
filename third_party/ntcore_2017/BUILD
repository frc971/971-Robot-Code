licenses(['notice'])

cc_library(
  name = 'ntcore',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/**/*.cpp',
    'src/**/*.h',
    'wpiutil/src/**/*.cpp',
  ], exclude = [
    'src/networktables/**',
  ]),
  copts = [
    '-Wno-switch-enum',
    '-Wno-cast-align',
  ],
  hdrs = glob([
    'include/**/*.h',
    'wpiutil/include/**/*.h',
  ]),
  includes = [
    'include',
    'wpiutil/include',
  ],
  linkopts = [
    '-lpthread',
  ],
)
