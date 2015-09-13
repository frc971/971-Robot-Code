$LOAD_PATH.unshift(".")

["tokenizer.rb","q_file.rb","queue_group.rb","queue.rb","namespaces.rb",
 "interface.rb","errors.rb", "q_struct.rb"].each do |name|
  require_relative "objects/" + name
end
["standard_types.rb","auto_gen.rb","file_pair_types.rb",
  "dep_file_pair.rb"].each do |name|
	require_relative "cpp_pretty_print/" + name
end
["q_file.rb","message_dec.rb","queue_dec.rb", "q_struct.rb"].each do |name|
	require_relative "output/" + name
end
require_relative "write_iff_changed.rb"

require "fileutils"
require "pathname"
