require 'set'
require_relative "../point2d.rb"

class Line2d
	init_fields :st, :ed
end

class Arc2d
	init_fields :c, :r, :sta, :eda
	def pt_for(a) @c + Point2d.new(@r * Math.cos(a), @r * Math.sin(a)) end
	def st() pt_for(@sta) end
	def ed() pt_for(@eda) end
end

class CircleProfile2d
	init_fields :c, :r
	def prepare_sweepline()
	end
end


class LoopProfile2d
	def initialize(items)
		@items = items
	end
end

class PtRef
	init_fields :pt, :obj, :key
	attr_accessor :refpair, :pair
	def inspect() "#{@pt.inspect}" end
	def comp_v(o) @pt.comp_v(o.pt) end
	def comp_h(o) @pt.comp_h(o.pt) end
	def self.add_pair(item, k, pts)
		pts << a = PtRef.new(item.st, k, pts)
		pts << b = PtRef.new(item.ed, item, k)
		a.refpair = b
		b.refpair = a
	end
	def self.add_twin_pair(item, k, pts)
		pts << a = PtRef.new(item.st, item, k)
		pts << b = PtRef.new(item.twin.st, item.twin, k)
		a.refpair = b
		b.refpair = a
	end
end

def point_ref_cluster_x(pts, broken, tolerance) 
	return if (pts.length == 0)
	pts.sort!() { |a, b| a.comp_h(b) } 
	
	broken_pts = []
	old_pt_pt_x = pts[0].pt.x

	pts.each do |pt|
		if (pt.pt.x - old_pt_pt_x > tolerance)
			broken << broken_pts
			broken_pts = []
		end
		broken_pts << pt
		old_pt_pt_x = pt.pt.x
	end
	broken << broken_pts
end

def point_ref_cluster_y(pts, broken, tolerance) 
	return if (pts.length == 0)
	pts.sort!() { |a, b| a.comp_v(b) } 
	
	broken_pts = []
	old_pt_pt_y = pts[0].pt.y

	pts.each do |pt|
		if (pt.pt.y - old_pt_pt_y > tolerance)
			broken << broken_pts
			broken_pts = []
		end
		broken_pts << pt
		old_pt_pt_y = pt.pt.y
	end
	broken << broken_pts
end


def clock_wise_cluster(a, b)
	#puts " --- #{a.inspect} --- #{b.inspect} --- "
	if a.obj.st.comp_v(a.obj.ed) != a.obj.st.comp_v(b.obj.ed)
		flip = a.obj.ed.comp_v(b.obj.ed)
	else
#		puts orient2d(a.obj.st, a.obj.ed, b.obj.ed)
	end
	return flip > 0 ? b : a
end

def point_ref_cluster(pts, tolerance)
	pts = [pts]
	broken = []
	pts.each do |segment|
		point_ref_cluster_x(segment, broken, tolerance)
	end
	pts = broken
	broken = []
	pts.each do |segment|
		point_ref_cluster_y(segment, broken, tolerance)
	end
	return broken
end

def point_ref_cluster_and_validate(pts, tolerance)
	clusters = point_ref_cluster(pts, tolerance)

	clusters.each do |cluster|
		if (cluster.length > 2)
			raise "profile features too fine for tolerance = #{tolerance}" 
		end
		if (cluster.length < 2)
			#raise "sketch has open contours..."
			cluster[0].pair = cluster[0]
		else
		cluster[0].pair = cluster[1]
		cluster[1].pair = cluster[0]
		end
		#cluster[0].pair = cluster[1]
		#cluster[1].pair = cluster[0]
	end
	clusters.sort! { |a, b|
		a[0].comp_h(b[0])
	}

	return clusters
end

def join_entities_into_loop_graph(item_list, tolerance)
	profiles = []
	pts = []

	k = 0
	item_list.each do |item|
		if (item.respond_to?(:st))
			PtRef.add_pair(item, k, pts)
			k += 1
		else
			profiles << item
		end
	end

	clusters = point_ref_cluster_and_validate(pts, tolerance)

	visited = []

	pts.each do |ptref|
		if (!visited[ptref.key])
			sub_profile = []
			while !visited[ptref.key]
				visited[ptref.key] = true
				sub_profile << ptref
				ptref = ptref.pair.refpair
			end
			profiles << LoopProfile2d.new(sub_profile)
		end
	end

	return profiles
end

require 'algorithms'
require "containers/priority_queue.rb"

class ForkEvent
	attr_accessor :profile_id
	init_fields :pt, :obj1, :obj2
	def draw_to(cr, scale)
		cr.set_source_rgb(0.0, 1.0, 0.0)
		cr.arc(@pt.x * scale, - @pt.y * scale, 0.05 * scale, 0, 6.25)
		cr.fill()
	end
end

class MergeEvent
	attr_accessor :profile_id
	init_fields :pt, :obj1, :obj2
	def draw_to(cr, scale)
		cr.set_source_rgb(1.0, 0.0, 0.0)
		cr.arc(@pt.x * scale, - @pt.y * scale, 0.05 * scale, 0, 6.25)
		cr.fill()
	end
end

class ChainEvent
	attr_accessor :profile_id
	init_fields :pt, :obj_in, :obj_out
	def draw_to(cr, scale)
		cr.set_source_rgb(1.0, 0.0, 0.0)
		cr.arc(@pt.x * scale, - @pt.y * scale, 0.05 * scale, 0, 6.25)
		cr.fill()
	end
end

class EventObj
	def fork_event(other) ForkEvent.new(st(), self, other)  end
	def merge_event(other) MergeEvent.new(ed(), self, other) end
	def st_chain_event(other) ChainEvent.new(st(), self, other) end
	def ed_chain_event(other) ChainEvent.new(ed(), other, self) end
end

class UpperArc < EventObj
	init_fields :sta, :eda, :c, :r
	def pt_for(a) @c + Point2d.new(@r * Math.cos(a), @r * Math.sin(a)) end
	def st() pt_for(@sta) end
	def ed() pt_for(@eda) end
end

class LowerArc < EventObj
	init_fields :sta, :eda, :c, :r
	def pt_for(a) @c + Point2d.new(@r * Math.cos(a), @r * Math.sin(a)) end
	def st() pt_for(@sta) end
	def ed() pt_for(@eda) end
end

class LinePt < EventObj
	init_fields :st, :ed
end

module Math; PI2 = Math::PI * 2; end

class Arc2d
	def get_add_events_to(sweep, i)
		if (@sta < Math::PI && @eda > Math::PI)
			larc = UpperArc.new(Math::PI, @sta, @c, @r)
			uarc = LowerArc.new(-Math::PI, @eda - Math::PI2, @c, @r)
			puts "arc2d: #{@sta / Math::PI} #{@eda / Math::PI}"

			sweep.add_event(larc.fork_event(uarc), i)
		end
		if (@eda < @sta && @sta != 0 && @eda != 0)
			larc = UpperArc.new(@sta, 0, @c, @r)
			uarc = LowerArc.new(@eda - Math::PI2, 0, @c, @r)
			puts "arc2d: #{@sta / Math::PI} #{@eda / Math::PI}"

			sweep.add_event(larc.merge_event(uarc), i)
		end
	end
end

class Line2d
	def get_add_events_to(sweep, i)
		if (@st.comp_h(@ed) < 0)
			sweep.add_event(ChainEvent.new(@st, self, nil), i)
		else
			sweep.add_event(ChainEvent.new(@ed, self, nil), i)
		end
	end
end

class LoopProfile2d
	def add_events_to(sweep, i)
		@items.each do |item|
			item.obj.get_add_events_to(sweep, i)
		end
	end
end

class CircleProfile2d
	def add_events_to(sweep, i)
		obju = UpperArc.new(Math::PI, 0, @c, @r)
		objl = LowerArc.new(-Math::PI, 0, @c, @r)
		sweep.add_event(obju.fork_event(objl), i)
		sweep.add_event(obju.merge_event(objl), i)
	end
end

class SweepLineConnectivity
	class ArcSweepLinePoint

	end
	attr_accessor :events
	def initialize()
		@profile_ids = []
		@events = []
	end
	def add_event(evt, i)
		evt.profile_id = i
		@events << evt
	end
	def get_lex_min_profile()
		@events.sort! { |a, b| a.pt.comp_h(b.pt) }
		@profile_ids[@events[0].profile_id]
	end
	def set_profile_id(i, profile)
		@profile_ids[i] = profile
	end
end
