package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'aos_strsignal',
  srcs = [
    'aos_strsignal.cc',
  ],
  deps = [
    '//aos/common/logging:logging_interface',
  ],
)

cc_test(
  name = 'aos_strsignal_test',
  srcs = [
    'aos_strsignal_test.cc',
  ],
  deps = [
    ':aos_strsignal',
    '//third_party/gtest',
    '//aos/common/logging:logging',
  ],
)

cc_library(
  name = 'dirname',
  srcs = [
    'dirname.cc',
  ],
)

cc_test(
  name = 'dirname_test',
  srcs = [
    'dirname_test.cc',
  ],
  deps = [
    '//third_party/gtest',
  ],
)

cc_library(
  name = 'aos_strerror',
  srcs = [
    'aos_strerror.cc',
  ],
)

cc_test(
  name = 'aos_strerror_test',
  srcs = [
    'aos_strerror_test.cc',
  ],
  deps = [
    ':aos_strerror',
    '//third_party/gtest',
  ],
)
