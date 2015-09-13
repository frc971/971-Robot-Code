load('/tools/build_rules/ruby', 'ruby_library', 'ruby_binary')

ruby_library(
  name = 'lib',
  srcs = [
    'load.rb',
    'write_iff_changed.rb',
  ] + glob(['*/*.rb']),
)

ruby_binary(
  name = 'compiler',
  visibility = ['//visibility:public'],
  srcs = [
    'compiler.rb',
  ],
  deps = [
    ':lib',
  ],
)

ruby_binary(
  name = 'queue_primitives',
  visibility = ['//visibility:public'],
  srcs = [
    'queue_primitives.rb',
  ],
  deps = [
    ':lib',
  ],
)

ruby_binary(
  name = 'print_field',
  visibility = ['//visibility:public'],
  srcs = [
    'print_field.rb',
  ],
  deps = [
    ':lib',
  ],
)
