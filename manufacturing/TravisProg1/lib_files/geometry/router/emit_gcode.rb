require "geometry/gcode/parse.rb"
require "geometry/gcode/gcode_file_format.rb"
require "geometry/gcode/gcode_layout.rb"

class GcodeLoader
	def initialize() @cache = {} end
	def load_pm(fname)
		@cache[fname] ||= interpret_gcode(load_gcode(fname))
	end
end

module Router
	class HoleSpec
		def self.emit_gcode_group(item, gcode_group)
			gcode_group.add_hole(item)
		end
	end
	class GcodeSpec
		def emit_gcode_group(item, gcode_group)
			@gcode_files.each do |gcode_file|
				ext = gcode_file.basename.to_path[0...-gcode_file.extname.length].split("_")[-1]
				gcode_group.add_part(ext, gcode_file, item)
			end
			gcode_group
		end
	end
	class GcodeGrouping
		def initialize()
			@suffix_groups = {}
			@holes = []
		end
		def emit_group(group, fname, origin)
			loader = GcodeLoader.new()
			lay = GcodeLayout.new()
			group.each do |pos, gcode|
				lay.add_item(loader.load_pm(gcode), pos.c - origin, pos.rot)
			end
			lay.write_gcode(fname)
		end
		def emit_holes(fname, sheet_info, origin)
			loader = GcodeLoader.new()
			lay = GcodeLayout.new()
			hole_fname = sheet_info.hole_file()
			@holes.each do |pos|
				lay.add_item(loader.load_pm(hole_fname), pos.c - origin, pos.rot)
			end
			lay.write_gcode(fname)
		end
		def groups() @suffix_groups end
		def add_part(ext, gcode_file, item)
			(@suffix_groups[ext] ||= []) << [item, gcode_file]
		end
		def add_hole(hole)
			@holes << hole
		end
		def has_holes() @holes.length > 0 end
	end
	class SheetCut
		def make_gcode_groupings()
			gcode_group = GcodeGrouping.new()
			@layout.items.each do |item|
				item.item.filename.emit_gcode_group(item, gcode_group)
			end
			return gcode_group
		end
	end
end
