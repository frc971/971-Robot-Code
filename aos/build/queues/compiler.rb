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
require "fileutils"
require "pathname"

def parse_args(globals,args)
	i = 0
	while(i < args.length)
		if(args[i] == "-I")
			args.delete_at(i)
			if(!args[i])
				$stderr.puts "hey! -I is followed by nothing."
				$stderr.puts "\tnot a supported usage..."
				$stderr.puts "\tWot. Wot."
				exit!(-1)
			end
			path = args.delete_at(i)
			globals.add_path(path)
		elsif(args[i] == "-cpp_out")
			args.delete_at(i)
			path = args.delete_at(i)
			if(path =~ /\./)
				$stderr.puts "hey! path #{path} has a \".\" char which is "
				$stderr.puts "\tnot a supported usage..."
				$stderr.puts "\tWot. Wot."
				exit!(-1)
			elsif(!path)
				$stderr.puts "hey! No cpp_out path provided."
				$stderr.puts "\tumm, you could try -cpp_out \"\""
				$stderr.puts "\tThat might do the trick"
				$stderr.puts "\tWot. Wot."
				exit!(-1)
			end
			$cpp_out = path.split(/\\|\//)
		elsif(args[i] == "-cpp_base")
			args.delete_at(i)
			path = args.delete_at(i)
			$cpp_base = File.expand_path(path)
			if(!File.exists?($cpp_base))
				$stderr.puts "output directory #{$cpp_base.inspect} does not exist."
				$stderr.puts "\tI'm not going to make that! sheesh, who do you think I am?"
				$stderr.puts "\tWot. Wot."
				exit!(-1)
			end
		elsif(args[i] =~ /^-/)
			$stderr.puts "hey! unknown argument #{args[i]}."
			$stderr.puts "\tWot. Wot."
			exit!(-1)
		else
			i += 1
		end
	end
	if(!$cpp_base)
		$stderr.puts "hey! missing -cpp_base argument."
		$stderr.puts "\tWot. Wot."
		exit!(-1)
	end
	if(!$cpp_out)
		$stderr.puts "hey! missing -cpp_out argument."
		$stderr.puts "\tWot. Wot."
		exit!(-1)
	end
end
def build(filename,globals_template)
	globals = Globals.new()
	globals_template.paths.each do |path|
		globals.add_path(path)
	end
	filename = File.expand_path(filename)
	q_file = QFile.parse(filename)
	output_file = q_file.q_eval(globals)
	q_filename = File.basename(filename)
	rel_path = ($cpp_out + [q_filename]).join("/")

	FileUtils.mkdir_p(Pathname.new($cpp_base) + $cpp_out.join("/"))

	cpp_tree = output_file.make_cpp_tree(rel_path)

	h_file_path = $cpp_base + "/" + rel_path + ".h"
	cc_file_path = $cpp_base + "/" + rel_path + ".cc"
	cpp_tree.add_cc_include((rel_path + ".h").inspect)
	cpp_tree.add_cc_include("aos/common/byteorder.h".inspect)
	cpp_tree.add_cc_include("aos/common/inttypes.h".inspect)
	cpp_tree.add_cc_include("aos/common/queue_types.h".inspect)
  cpp_tree.add_cc_include("aos/common/once.h".inspect)
	cpp_tree.add_cc_using("::aos::to_network")
	cpp_tree.add_cc_using("::aos::to_host")

	header_file = File.open(h_file_path,"w+")
	cc_file = File.open(cc_file_path,"w+")
	cpp_tree.write_header_file($cpp_base,header_file)
	cpp_tree.write_cc_file($cpp_base,cc_file)
	cc_file.close()
	header_file.close()
end
begin
	args = ARGV.dup
	globals = Globals.new()
	parse_args(globals,args)
	if(args.length == 0)
		$stderr.puts "hey! you want me to do something,"
		$stderr.puts "\tbut you gave me no q files to build!"
		$stderr.puts "\tWot. Wot."
		exit!(-1)
	end
	args.each do |filename|
		build(filename,globals)
	end
	exit(0)
rescue QError => e
	$stderr.print(e.to_s)
	exit!(-1)
end
