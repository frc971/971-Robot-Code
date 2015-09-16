load('/tools/build_rules/ruby', 'ruby_binary')

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
