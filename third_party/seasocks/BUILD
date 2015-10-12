licenses(['notice'])

cc_library(
  name = 'seasocks',
  visibility = ['//visibility:public'],
  srcs = glob(['src/main/c/**/*.cpp']),
  hdrs = glob([
    'src/main/c/**/*.h',
  ], [
    'src/main/internal/**/*',
  ]),
  includes = ['src/main/c'],
  copts = [
    # TODO(Brian): Don't apply this to all of the code...
    '-Wno-cast-align',
    '-Wno-cast-qual',

    '-Wno-switch-enum',
    '-Wno-format-nonliteral',
    '-Wno-unused-parameter',
  ],
)
