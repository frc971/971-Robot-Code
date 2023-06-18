require "fileutils"
require "pathname"
require "set"

module Router
	class GcodeSpec
		class SpecError
			def initialize(msg)
				@msg = msg
			end
			def msg() @msg end
		def is_error() true end
		end
		def is_error() false end
		def initialize(folder, dxf, gcode_files)
			@folder = folder
			@dxf = dxf
			@gcode_files = gcode_files
			puts self.inspect()
		end
		def dxf_name() @folder + @dxf end
		def inspect()
			"<Part: #{@folder.to_path.inspect}, #{@dxf.inspect}, #{@gcode_files.inspect}>"
		end
		def name()
			@folder.dirname.to_path.inspect
		end
		def full_path() @folder.to_path end
		def self.make(folder, children)
			# puts "#{folder.inspect} -> #{children.inspect}"
			dxfs = []
			ngcs = []
			children.each do |child|
				dxfs << child if (child.extname == ".dxf")
				dxfs << child if (child.extname == ".DXF")
				ngcs << child if (child.extname == ".ngc")
			end
			return nil if (dxfs.length == 0 && ngcs.length == 0)
			return SpecError.new("#{folder.inspect} #{dxfs.length} dxfs!") if (dxfs.length != 1)
			return SpecError.new("#{folder.inspect} no ngcs!") if (ngcs.length == 0)
			return GcodeSpec.new(folder, dxfs[0].basename.to_path, ngcs)
		end
	end
	class HoleSpec
		Radius = 0.25
	end
	class PartsLibrary
		def fname() @folder.basename.to_path end
		def initialize(folder)
			@folder = folder
			@sub_libs_by_name = {}
			rescan()
		end
		def rescan()
			#puts "entering #{fname()}"
			c = @folder.children
			bad = c.select() { |a| !(a.directory? || a.file?) }
			raise "bad filenames of #{fname()} -> #{bad}" if (!bad.empty?()) 
			@spec = GcodeSpec.make(@folder, c.select(&:file?))
			@sub_libs = [] 
			c.select(&:directory?).each do |d| 
				subl = PartsLibrary.new(d)
				if subl.non_empty()
					if subl2 = @sub_libs_by_name[subl.fname]
						subl2.rescan()
						subl = subl2
					else
						puts "storing: #{subl.fname.inspect}"
						@sub_libs_by_name[subl.fname] = subl
					end
					@sub_libs << subl
				end
			end
		end
		def sub_libs() @sub_libs end
		def spec() @spec end
		def non_empty()
			@spec || @sub_libs.length != 0
		end
		def realize(fname)
			# puts "fname: #{fname.inspect} #{@folder.inspect}"
			deltp = fname.relative_path_from(@folder)
			key = get_key_from(deltp)
			# puts "fname: #{deltp} -> key: #{key}"
			if (key)
				#puts "key: #{key.inspect}"
				target = @sub_libs_by_name[key]
				if (target)
					return @sub_libs_by_name[key].realize(fname)
				else
					puts "cannot find #{key} in #{@sub_libs_by_name.keys.inspect}"
					return nil
				end
			else
				if (@spec.is_error)
					puts "realizing error parts library spec: #{fname} err: #{@spec.msg}"
				end
				return (!@spec || @spec.is_error) ? nil : @spec
			end
			#raise "problem!"
		end
		def get_key_from(deltp)
			goal = Pathname.new(".")
			return nil if (deltp == goal)
			while deltp.dirname != goal
				deltp = deltp.dirname
			end
			return deltp.to_path
		end
	end
end
