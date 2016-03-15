cc_library(
  name = 'vector',
  visibility = ['//visibility:public'],
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
