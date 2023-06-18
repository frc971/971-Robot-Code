require "geometry/dxf_to_gcode/2dpoint.rb"
require_relative "offset_profiles.rb"

require "geometry/dxf_to_gcode/dxf.rb"
require_relative "dxf_to_offset_profiles.rb"

require "geometry/dxf_to_gcode/timed_view.rb"
require_relative "overlay.rb"
require_relative "pairwise_inter.rb"

def offset_dxf_integer(dxf, scale, resolution, offset)
	lns = dxf[0].to_offsetable(scale)

	all_lns = []
	lns.each do |ln|
		rlns = ln.offset(((offset) * scale).to_i)
		segments = []
		rlns.each do |rln|
			rln.append_to(segments, resolution)
		end
		shape = Shape.new(segments)
		all_lns << shape
	end

#	swl_p, nticks = SweepLineUnion.make_outside_profile(all_lns) 
	swl_p = AllPairsInter.inter(all_lns) 

	return swl_p#, all_lns
end
