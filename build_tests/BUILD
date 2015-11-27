load('/tools/build_rules/ruby', 'ruby_binary')
load('/aos/build/queues', 'queue_library')

cc_test(
  name = 'gflags_build_test',
  srcs = [
    'gflags.cc',
  ],
  deps = [
    '//third_party/gflags',
  ],
  size = 'small',
)

ruby_binary(
  name = 'ruby_binary',
  srcs = [
    'ruby.rb',
    'ruby_to_require.rb',
  ],
  data = [
    'ruby_to_require.rb',
  ],
)

sh_test(
  name = 'ruby_build_test',
  srcs = [
    'ruby_check.sh',
  ],
  data = [
    ':ruby_binary',
  ],
  size = 'small',
)

queue_library(
  name = 'queue_library',
  srcs = [
    'queue.q',
  ],
)

cc_test(
  name = 'queue_build_test',
  srcs = [
    'queue.cc',
  ],
  deps = [
    ':queue_library',
  ],
  size = 'small',
)

cc_binary(
  name = 'tcmalloc_build_test_binary',
  srcs = [
    'tcmalloc.cc',
  ],
)

sh_test(
  name = 'tcmalloc_build_test',
  srcs = [
    'tcmalloc_test.sh',
  ],
  data = [
    ':tcmalloc_build_test_binary',
  ],
  size = 'small',
)
