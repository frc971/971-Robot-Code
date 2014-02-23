$LOAD_PATH.unshift(".")
["tokenizer.rb","q_file.rb","queue_group.rb","queue.rb","namespaces.rb",
"interface.rb","errors.rb", "q_struct.rb"].each do |name|
	require File.dirname(__FILE__) + "/objects/" + name
end
["standard_types.rb","auto_gen.rb","file_pair_types.rb",
"dep_file_pair.rb"].each do |name|
	require File.dirname(__FILE__) + "/cpp_pretty_print/" + name
end
["q_file.rb","message_dec.rb","queue_dec.rb", "q_struct.rb"].each do |name|
	require File.dirname(__FILE__) + "/output/" + name
end
require File.dirname(__FILE__) + '/write_iff_changed.rb'
require "fileutils"
require "pathname"
