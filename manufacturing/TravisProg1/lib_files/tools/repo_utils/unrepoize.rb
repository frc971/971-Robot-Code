def unrepoize(extern_path, target)
	if ARGV[0] == "-unrepoize"
		$LOAD_PATH[0] = "~/repo"
		require 'fileutils'
		ARGV.slice!(0..0)
		extern_path = File.expand_path(extern_path)
		repo_path = File.expand_path("~/repo")
		target = extern_path + target
		exp = [File.expand_path("~/repo/tools/repo_utils/unrepoize.rb")]
		
#		puts target
		$unrepoize_handler = Proc.new() {
			(exp + $").each do |f|
				#f = String.new(f)
				#puts f.inspect, f.class
#				FileUtils.mkpath(target)
				if (f.start_with?(repo_path) && !f.start_with?(extern_path))
					sf = f
					tf = target + f[repo_path.length..-1]
					FileUtils.mkpath(File.dirname(tf))
					FileUtils.cp(sf, tf)
					#puts "unrepoize #{tf}"
				else
					#puts "skipping: #{f.inspect}"
				end
			end
		}
	end
end

def unrepoize_finish()
	if $unrepoize_handler
	 	$unrepoize_handler.call()
	end
end
