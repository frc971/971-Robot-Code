cc_library(
  name = 'interpolation',
  hdrs = [
    'interpolation.h',
  ],
  srcs = [
    'interpolation.cc',
  ],
  visibility = ['//visibility:public'],
)

cc_test(
  name = 'interpolation_test',
  srcs = [
    'interpolation_test.cc',
  ],
  deps = [
   ':interpolation',
   '//aos/testing:googletest',
  ]
)
