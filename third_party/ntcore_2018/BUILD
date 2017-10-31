licenses(['notice'])

cc_library(
  name = 'ntcore',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/main/**/*.cpp',
    'src/main/**/*.h',
  ], exclude = [
    #'src/main/native/cpp/networktables/**',
    'src/main/native/cpp/jni/**',
  ]),
  copts = [
    '-Wno-switch-enum',
    '-Wno-cast-align',
  ],
  hdrs = glob([
    'src/main/native/include/**/*.h',
    'src/main/native/include/**/*.inl',
  ]),
  includes = [
    'src/main/native/include',
  ],
  linkopts = [
    '-lpthread',
  ],
  deps = [
    '//third_party/wpiutil_2018:wpiutil',
  ],
  restricted_to = ['//tools:roborio'],
)
