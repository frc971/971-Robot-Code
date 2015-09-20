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
)
