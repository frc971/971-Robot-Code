# ruby_check.sh runs this and verifies the output matches.
# The first argument is the absolute path to the runfiles directory to use for
# testing require with an absolute path.

$loaded = false
raise unless require_relative 'ruby_to_require.rb'
raise unless $loaded

# We already loaded this above, so it won't happen again.
raise if require 'build_tests/ruby_to_require'

$loaded = false
raise unless require ARGV[0] + '/build_tests/ruby_to_require'
raise unless $loaded

raise unless ''.encoding == Encoding::UTF_8

puts 'Hi from ruby'
