require_relative "entities.rb"
class TwinableProfile
	def draw_to(cr, scale)
		@items.each do |item|
			cr.set_source_rgb(rand(), rand(), rand())
			item.seg.draw_to(cr, scale)
		end
	end
	def draw_path_to(cr, scale)
		draw_to(cr, scale)
		return;
	#	@items[0].draw_start_path(cr, scale)
		@items.each do |item|
			item.draw_path_to(cr, scale)
		end
		cr.stroke()
	end
	def draw_direction_to(cr, scale)
		@items.each do |item|
			item.draw_direction_to(cr, scale)
		end
	end
end

class SegmentRef
	class ReversedSegRef
		def draw_to(cr, scale)
			@seg.draw_to(cr, scale)
			draw_direction_to(cr, scale)
		end
		def draw_start_path(cr, scale)
			@seg.draw_rev_start_path(cr, scale)
		end
		
		def draw_path_to(cr, scale)
			@seg.draw_rev_path_to(cr, scale)
		end
		def draw_direction_to(cr, scale)
			mp, tang = @seg.midpoint_and_tangent()
	#		cr.set_source_rgb(0, 1, 1)
			tang = tang.scale(-0.05)
			arrt = 0.5
			cr.move_to((mp.x + tang.x) * scale, -(mp.y + tang.y) * scale)
			cr.line_to((mp.x - arrt * tang.y) * scale, -(mp.y + arrt * tang.x) * scale)
			cr.line_to((mp.x + arrt * tang.y) * scale, -(mp.y - arrt * tang.x) * scale)
			cr.line_to((mp.x + tang.x) * scale, -(mp.y + tang.y) * scale)
			cr.fill()
		end
	end
	def draw_path_to(cr, scale)
		@seg.draw_path_to(cr, scale)
	end
	def draw_start_path(cr, scale)
		@seg.draw_start_path(cr, scale)
	end
	def draw_to(cr, scale)
		@seg.draw_to(cr, scale)
		draw_direction_to(cr, scale)
	end
	def draw_direction_to(cr, scale)
		mp, tang = @seg.midpoint_and_tangent()
#		cr.set_source_rgb(0, 1, 1)
		tang = tang.scale(0.05)
		arrt = 0.5
		cr.move_to((mp.x + tang.x) * scale, -(mp.y + tang.y) * scale)
		cr.line_to((mp.x - arrt * tang.y) * scale, -(mp.y + arrt * tang.x) * scale)
		cr.line_to((mp.x + arrt * tang.y) * scale, -(mp.y - arrt * tang.x) * scale)
		cr.line_to((mp.x + tang.x) * scale, -(mp.y + tang.y) * scale)
		cr.fill()
	end
end

class QuadrentArc
	def draw_to(cr, scale)
#		puts "arclen: #{@eda - @sta} "
		cr.arc(@c.x * scale, - @c.y * scale, @r * scale, -@eda, -@sta)
		cr.stroke()
	end
	def draw_path_to(cr, scale)
		cr.arc_negative(@c.x * scale, - @c.y * scale, @r * scale, -@sta, -@eda)
		#cr.stroke()
	end
	def draw_rev_path_to(cr, scale)
		cr.arc(@c.x * scale, - @c.y * scale, @r * scale, -@eda, -@sta)
	end
end

class Line2d
	def draw_to(cr, scale)
		cr.move_to(@st.x * scale, - @st.y * scale)
		cr.line_to(@ed.x * scale, - @ed.y * scale)
		cr.stroke()
	end
	def draw_path_to(cr, scale)
		#cr.move_to(@st.x * scale, - @st.y * scale)
		cr.line_to(@ed.x * scale, - @ed.y * scale)
	end
	def draw_rev_path_to(cr, scale)
		#cr.move_to(@ed.x * scale, - @ed.y * scale)
		cr.line_to(@st.x * scale, - @st.y * scale)
	end
end
