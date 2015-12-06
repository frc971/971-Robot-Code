package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'run_command',
  srcs = [
    'run_command.cc',
  ],
  hdrs = [
    'run_command.h',
  ],
  deps = [
    '//aos/common/logging',
  ],
)

cc_test(
  name = 'run_command_test',
  srcs = [
    'run_command_test.cc',
  ],
  deps = [
    ':run_command',
    '//aos/testing:googletest',
    '//aos/common/logging',
    ':thread',
  ],
)

cc_library(
  name = 'death_test_log_implementation',
  hdrs = [
    'death_test_log_implementation.h',
  ],
  deps = [
    '//aos/common/logging:implementations',
  ],
)

cc_library(
  name = 'inet_addr',
  srcs = [
    'inet_addr.cc',
  ],
  hdrs = [
    'inet_addr.h',
  ],
  deps = [
    '//aos/common:byteorder',
    '//aos/common:network_port',
  ],
)

cc_library(
  name = 'phased_loop',
  srcs = [
    'phased_loop.cc',
  ],
  hdrs = [
    'phased_loop.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common:time',
  ],
)

cc_library(
  name = 'log_interval',
  hdrs = [
    'log_interval.h',
  ],
  deps = [
    '//aos/common:time',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'string_to_num',
  hdrs = [
    'string_to_num.h',
  ],
)

cc_test(
  name = 'string_to_num_test',
  srcs = [
    'string_to_num_test.cc',
  ],
  deps = [
    ':string_to_num',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'thread',
  srcs = [
    'thread.cc',
  ],
  hdrs = [
    'thread.h',
  ],
  deps = [
    '//aos/common:macros',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'trapezoid_profile',
  srcs = [
    'trapezoid_profile.cc',
  ],
  hdrs = [
    'trapezoid_profile.h',
  ],
  deps = [
    '//third_party/eigen',
    '//aos/common:time',
    '//aos/common/logging',
    '//debian:libm',
  ],
)

cc_test(
  name = 'trapezoid_profile_test',
  srcs = [
    'trapezoid_profile_test.cc',
  ],
  deps = [
    ':trapezoid_profile',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'wrapping_counter',
  srcs = [
    'wrapping_counter.cc',
  ],
  hdrs = [
    'wrapping_counter.h',
  ],
)

cc_test(
  name = 'wrapping_counter_test',
  srcs = [
    'wrapping_counter_test.cc',
  ],
  deps = [
    ':wrapping_counter',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'options',
  hdrs = [
    'options.h',
  ],
)

cc_test(
  name = 'options_test',
  srcs = [
    'options_test.cc',
  ],
  deps = [
    ':options',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'compiler_memory_barrier',
  hdrs = [
    'compiler_memory_barrier.h',
  ],
)

cc_library(
  name = 'linked_list',
  hdrs = [
    'linked_list.h',
  ],
  deps = [
    '//aos/common:transaction',
  ],
)

cc_test(
  name = 'linked_list_test',
  srcs = [
    'linked_list_test.cc',
  ],
  deps = [
    ':linked_list',
    '//aos/testing:googletest',
    '//aos/common/logging',
  ],
)

cc_test(
  name = 'phased_loop_test',
  srcs = [
    'phased_loop_test.cc',
  ],
  deps = [
    ':phased_loop',
    '//aos/testing:googletest',
    '//aos/testing:test_logging',
  ],
)
