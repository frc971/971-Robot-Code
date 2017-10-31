licenses(['notice'])

cc_library(
  name = 'wpiutil',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/main/native/cpp/**/*.cpp',
    'src/main/native/cpp/**/*.h',
    'src/main/native/cpp/**/*.inc',
  ]),
  copts = [
    #'-Wno-switch-enum',
    '-Wno-cast-align',
    #'-Ithird_party/wpiutil_2018/
    '-Wno-unused-parameter',
  ],
  hdrs = glob([
    'src/main/native/include/**/*',
  ]),
  includes = [
    'src/main/native/include',
  ],
  linkopts = [
    '-lpthread',
  ],
  restricted_to = ['//tools:roborio'],
)
