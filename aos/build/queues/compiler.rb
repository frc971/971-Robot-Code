require_relative "load.rb"

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
    elsif(args[i] == "-src_filename")
      args.delete_at(i)
      $src_filename = args.delete_at(i)
    elsif(args[i] == "-h_file_path")
      args.delete_at(i)
      path = args.delete_at(i)
      $h_file_path = File.expand_path(path)
      if(!File.exists?(File.dirname($h_file_path)))
        $stderr.puts "directory of output #{$h_file_path.inspect} does not exist."
        $stderr.puts "\tI'm not going to make that! sheesh, who do you think I am?"
        $stderr.puts "\tWot. Wot."
        exit!(-1)
      end
    elsif(args[i] == "-cc_file_path")
      args.delete_at(i)
      path = args.delete_at(i)
      $cc_file_path = File.expand_path(path)
      if(!File.exists?(File.dirname($cc_file_path)))
        $stderr.puts "directory of output #{$cc_file_path.inspect} does not exist."
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
  if ($cpp_base && $cpp_out) == ($src_filename && $h_file_path && $cc_file_path)
    $stderr.puts "hey! I'm not sure where to write the output files!"
    exit!(-1)
  end
end
def format_pipeline(output)
  read_in, write_in = IO.pipe()
  child = Process.spawn('/usr/bin/clang-format-3.5 --style=google',
                        {:in=>read_in, write_in=>:close,
                         :out=>output.fileno})
  read_in.close
  [child, write_in]
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

  if $cpp_base && $cpp_out
    $src_filename = ($cpp_out + [q_filename]).join("/")
    $h_file_path = $cpp_base + "/" + $src_filename + ".h"
    $cc_file_path = $cpp_base + "/" + $src_filename + ".cc"
    FileUtils.mkdir_p(Pathname.new($cpp_base) + $cpp_out.join("/"))
  end

  cpp_tree = output_file.make_cpp_tree($src_filename)

  cpp_tree.add_cc_include(($src_filename + ".h").inspect)
  cpp_tree.add_cc_include("aos/common/byteorder.h".inspect)
  cpp_tree.add_cc_include("<inttypes.h>")
  cpp_tree.add_cc_include("aos/common/queue_types.h".inspect)
  cpp_tree.add_cc_include("aos/common/once.h".inspect)
  cpp_tree.add_cc_include("aos/common/logging/printf_formats.h".inspect)
  cpp_tree.add_cc_using("::aos::to_network")
  cpp_tree.add_cc_using("::aos::to_host")

  header_file = WriteIffChanged.new($h_file_path)
  cc_file = WriteIffChanged.new($cc_file_path)
  header_child, header_output = format_pipeline(header_file)
  cc_child, cc_output = format_pipeline(cc_file)
  cpp_tree.write_header_file($cpp_base,header_output)
  cpp_tree.write_cc_file($cpp_base,cc_output)
  header_output.close()
  cc_output.close()
  if !Process.wait2(cc_child)[1].success?
    $stderr.puts "Formatting cc file failed."
    exit 1
  end
  if !Process.wait2(header_child)[1].success?
    $stderr.puts "Formatting header file failed."
    exit 1
  end
  header_file.close()
  cc_file.close()
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
