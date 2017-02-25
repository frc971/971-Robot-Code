load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')
package(default_visibility = ['//visibility:public'])

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
  name = 'socket_types',
  hdrs = ['socket_types.h'],
  deps = [
    '//aos/vision/events:tcp_server',
    '//aos/vision/image:image_types',
    '//third_party/protobuf:protobuf',
  ],
)

cc_library(
  name = 'intrusive_free_list',
  hdrs = ['intrusive_free_list.h'],
)

cc_library(
  name = 'tcp_server',
  srcs = ['tcp_server.cc'],
  hdrs = ['tcp_server.h'],
  deps = [':epoll_events', ':intrusive_free_list'],
)

cc_library(
  name = 'tcp_client',
  srcs = ['tcp_client.cc'],
  hdrs = ['tcp_client.h'],
  deps = [':epoll_events'],
)

cc_library(
  name = 'udp',
  srcs = ['udp.cc'],
  hdrs = ['udp.h'],
  deps = [
    '//aos/common:macros',
    '//aos/common:scoped_fd',
  ],
)

cc_test(
  name = 'udp_test',
  srcs = ['udp_test.cc'],
  deps = [
    ':udp',
    '//aos/testing:googletest',
  ],
)

gtk_dependent_cc_library(
  name = 'gtk_event',
  srcs = ['gtk_event.cc'],
  deps = [
    ':epoll_events',
    '@usr_repo//:gtk+-3.0',
  ],
)
