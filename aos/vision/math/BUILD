package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'segment',
  hdrs = ['segment.h'],
  deps = [':vector'],
)

cc_library(
  name = 'vector',
  hdrs = [
    'vector.h',
  ],
  deps = [
    '//third_party/eigen',
  ],
)

cc_test(
  name = 'vector_test',
  srcs = [
    'vector_test.cc',
  ],
  deps = [
    ':vector',
    '//aos/testing:googletest',
  ],
  size = 'small',
)
