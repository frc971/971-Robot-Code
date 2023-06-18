require "geometry/bool_ops/dxf_offset.rb"
IntegerScale = 2**20
class ProfileCrossing
	class Entity
		attr_accessor :st, :ed
		def initialize(st, ed) @st = st; @ed = ed; end
		def check_query(query)
			q = Point2di.new((query.x * IntegerScale).to_i, (query.y * IntegerScale).to_i)
			st = Point2di.new((@st.x * IntegerScale).to_i, (@st.y * IntegerScale).to_i)
			ed = Point2di.new((@ed.x * IntegerScale).to_i, (@ed.y * IntegerScale).to_i)
			return 0 if st.y == ed.y
			dir = 1
			st, ed, dir = ed, st, -1 if (st.y > ed.y)
			return 0 if (st.y > q.y || ed.y <= q.y)

			ttm = (q - st).cross(ed - st)
			return ttm > 0 ? dir : 0 
		end
	end
	def add_only_drawable(drawable)
		@only_drawable << drawable
	end
	attr_accessor :filename
	def initialize()
		@profile_entity = []
		@only_drawable = []
	end
	def query_crossings(query)
		crossings = 0
		@profile_entity.each do |entity|
			crossings += entity.check_query(query)
		end
		return crossings
	end
	def emit(st, ed)
		@profile_entity << Entity.new(st, ed)
	end
	def draw_to(cr, scale)
		item = @profile_entity[0]
		@profile_entity.each do |item|
			cr.move_to(item.st.x * scale, -item.st.y * scale)
			cr.line_to(item.ed.x * scale, -item.ed.y * scale)
			cr.stroke()
		end
		cr.set_source_rgb(1.0, 0.5, 0.0)
		@only_drawable.each do |item|
			item.items.each { |sitem| sitem.draw_to(cr, scale) }
		end
	end
end

class Line2d
	def emit_rev_crossing(st, crossing)
		crossing.emit(st, @st)
	end
	def emit_crossing(st, crossing)
		crossing.emit(st, @ed)
	end
end

class QuadrentArc
	def emit_rev_crossing(st, crossing)
		crossing.emit(st, self.st())
	end
	def emit_crossing(st, crossing)
		crossing.emit(st, self.ed())
	end
end

class SegmentRef
	class ReversedSegRef
		def emit_crossing(st, crossing)
			@seg.emit_rev_crossing(st, crossing)
		end
	end
	def emit_crossing(st, crossing)
		@seg.emit_crossing(st, crossing)
	end
end

class TwinableProfile
	def to_crossing(crossing)
		@items.length.times do |i|
			item = @items[i] 
			item.emit_crossing(@items[i - 1].ed, crossing)
		end
	end
end


class Shape
	# not correct if there is only ever two points. This is silly though.
	def to_crossing(scale, pm, res = 1)
		res_sqr = res * res
		st = @segs[0].st
		n = 0
		@segs.each do |seg|
			v = (seg.ed - st).mag_sqr.to_i
			if (v > res_sqr)
				st = seg.ed
				n += 1
			end
		end
		raise "problem" if n <= 2
		dnom = scale ** -1
		@segs.each do |seg|
			v = (seg.ed - st).mag_sqr.to_i
			if (v > res_sqr)
				ed = seg.ed
				pm.emit(ed * dnom, st * dnom)
			end
			st = seg.ed
		end
	end
end

def offset_dxf(dxf, scale, resolution, pm)
	swl_p = offset_dxf_integer(dxf, IntegerScale, 2**15)
end

obj_name = (ARGV[0] || "9971-15-P-0015_transmition_plate.dxf")
class PMLoader
	def initialize()
		@cache = {}
	end
	def direct_load(fname)
		items = get_entities(fname)
		items2d = items.collect { |item| item.to_2d() }

		items2d_orient = Oriented.import_2d(items2d)
		items2d_orient = SweepLine.stitch_curves(items2d_orient)
		items = items2d_orient

		scale = IntegerScale
		swl_p = offset_dxf_integer(items, scale, 2**15)
		pm = ProfileCrossing.new()
		pm.filename = fname
		swl_p.to_crossing(scale, pm)
		for item in items
			pm.add_only_drawable(item)
		end
		return pm
	end
	def load_pm(fname)
		@cache[fname] ||= direct_load(fname)
	end
end
