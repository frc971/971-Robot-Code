cc_library(
  name = 'epoll_events',
  srcs = ['epoll_events.cc'],
  hdrs = ['epoll_events.h'],
  deps = [
    '//aos/common:scoped_fd',
    '//aos/common/logging',
    '//aos/common:time',
  ],
)

cc_library(
  name = 'udp',
  visibility = ['//visibility:public'],
  srcs = ['udp.cc'],
  hdrs = ['udp.h'],
  deps = [
    '//aos/common:macros',
    '//aos/common/logging',
    '//aos/common:scoped_fd',
  ],
)

cc_test(
  name = 'udp_test',
  srcs = ['udp_test.cc'],
  deps = [
    ':udp',
    '//aos/testing:googletest'
  ],
)
