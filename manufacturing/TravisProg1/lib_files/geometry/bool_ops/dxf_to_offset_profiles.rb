
class Line2d
	def to_offsetable(st, scale)
		st = CompGeo::Point2di.new((st.x * scale).to_i, (st.y * scale).to_i)
		ed = CompGeo::Point2di.new((@ed.x * scale).to_i, (@ed.y * scale).to_i)
		return OffsetableLine.new(st, ed)
	end
	def to_offsetable_rev(ed, scale)
		st = CompGeo::Point2di.new((@st.x * scale).to_i, (@st.y * scale).to_i)
		ed = CompGeo::Point2di.new((ed.x * scale).to_i, (ed.y * scale).to_i)
		return OffsetableLine.new(st, ed)
	end
end

class QuadrentArc
	def to_offsetable(st, scale)
		st_ed_offsettable(st, ed, scale)
	end
	def to_offsetable_rev(ed, scale)
		st_ed_offsettable(st, ed, scale)
	end
	def st_ed_offsettable(st, ed, scale)
		st = CompGeo::Point2di.new((st.x * scale).to_i, (st.y * scale).to_i)
		ed = CompGeo::Point2di.new((ed.x * scale).to_i, (ed.y * scale).to_i)
		c = CompGeo::Point2di.new((@c.x * scale).to_i, (@c.y * scale).to_i)
		OffsettableArc.new(st, ed, c, (r * scale).to_i)
	end
end

class SegmentRef
	class ReversedSegRef
		def to_offsetable(st, scale)
			@seg.to_offsetable_rev(st, scale)
		end
	end
	def to_offsetable(st, scale)
		@seg.to_offsetable(st, scale)
	end
end
class TwinableProfile
	def to_offsetable(scale)
		@items.length.times.collect do |i|
			item = @items[i] 
			item.to_offsetable(item.st, scale)
		end
	end
end
