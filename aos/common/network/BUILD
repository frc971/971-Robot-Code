package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'team_number',
  srcs = [
    'team_number.cc',
  ],
  hdrs = [
    'team_number.h',
  ],
  deps = [
    '//aos/linux_code:configuration',
    '//aos/common:once',
    '//aos/common/logging',
    '//aos/common/util:string_to_num',
  ],
)

cc_library(
  name = 'socket',
  srcs = [
    'receive_socket.cc',
    'send_socket.cc',
    'socket.cc',
  ],
  hdrs = [
    'receive_socket.h',
    'send_socket.h',
    'socket.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common:time',
    '//aos/common/util:inet_addr',
    '//aos/linux_code:configuration',
  ],
)
