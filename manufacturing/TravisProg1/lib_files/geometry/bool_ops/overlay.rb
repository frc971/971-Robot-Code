require_relative "predicate.rb"

class Shape
	class Segment
		init_fields :st, :ed
	end
	attr_accessor :segs
	def initialize(segs)
		@segs = segs
	end
	def draw_blank_to(cr, scale)
#		cr.move_to(@segs[0].st.x * scale, -@segs[0].st.y * scale)
		@segs.each do |seg|
			cr.move_to(seg.st.x * scale, -seg.st.y * scale)
			cr.line_to(seg.ed.x * scale, -seg.ed.y * scale)
		end
		cr.stroke()
	end
	def draw_to(cr, scale)
#		cr.move_to(@segs[0].st.x * scale, -@segs[0].st.y * scale)
		i = 0
		@segs.each do |seg|
			k = (1.0 * i) / (@segs.length - 1)
			cr.set_source_rgb(1.0, k, 1.0 - k)
			cr.move_to(seg.st.x * scale, -seg.st.y * scale)
			cr.line_to(seg.ed.x * scale, -seg.ed.y * scale)
			cr.stroke()
			mp = (seg.st + seg.ed) * (2**-1 * scale)
			v = (seg.ed - seg.st)
			l = v.mag_sqr().to_i
			if (l > 1000)
			puts "#{v.mag_sqr()} -> #{v.mag_sqr().to_i}" if v.mag_sqr().to_i == 0
			v = v * (integer_sqrt(v.mag_sqr().to_i * 2**10)**-1 * 2**5)
			#puts "#{mp.x.inspect} #{mp.y.inspect}"
			#puts "#{v.x.inspect} #{v.y.inspect}"
			s1 = mp - v * 10 - v.rot90() * 5
			s2 = mp - v * 10 + v.rot90() * 5
			cr.move_to(mp.x, -mp.y)
			cr.line_to(s1.x, -s1.y)
			cr.line_to(s2.x, -s2.y)
			cr.line_to(mp.x, -mp.y)
			cr.stroke()
			end
			i += 1
		end
	end
	def add_pts(pts)
		@segs.each do |seg|
			pts << seg.st
		end
	end
	def add_evts(evts)
		status_lines = @segs.collect do |seg|
			side = x_lex_order(seg.st, seg.ed)
			(side < 0) ? StatusLine.new(seg.st, seg.ed, 1) : StatusLine.new(seg.ed, seg.st, -1)
		end

		@segs.length.times do |i|
			pt = @segs[i].st
			to_add = []
			to_del = []
			((status_lines[i].side > 0) ? to_add : to_del) << status_lines[i]
			((status_lines[i - 1].side < 0) ? to_add : to_del) << status_lines[i - 1]
=begin
			to_add.each do |spt|
				puts "to_add: #{(spt.st - pt).inspect}"
			end
			to_del.each do |spt|
				puts "to_del: #{(spt.ed - pt).inspect}"
			end
=end
			evts.add_event(AddDelEvent.new(pt, to_add, to_del))
		end
	end
end

class EventQueue
	attr_accessor :event_limit
	def initialize()
		@events = []
	end
	def flip()
		@events.sort!() { |a, b| b <=> a }
		@lnevents = @events
		@events = []
	end
	def add_event(event)
		if @event_limit
			return if @event_limit && (event <=> @event_limit) <= 0
		end
		@events << event
	end
	def pop_min_helper()
		@lnevents.sort!() { |a, b| b <=> a }
		m1 = @lnevents[-1]
		m2 = @events[-1]
		return @lnevents.pop() if !m2
		return @events.pop() if !m1
		d = m1 <=> m2
		#raise "events match!" if (d == 0)
		return (d < 0 ? @lnevents : @events).pop()
	end
	def pop_min()
		min = pop_min_helper()
		#puts "min: #{min}"
		while ((min2 = next_event()) && (min2 <=> min) == 0)
			min.merge_in(min2)
			pop_min()
		end
		return min
	end
	def next_event()
		@lnevents.sort!() { |a, b| b <=> a }
		m1 = @lnevents[-1]
		m2 = @events[-1]
		return m1 if !m2
		return m2 if !m1
		d = m1 <=> m2
		raise "events match!" if (d == 0)
		return d < 0 ? m1 : m2
	end
	def empty()
		@events.empty?() && @lnevents.empty?() 
	end
end

class StatusLine
	attr_accessor :st, :ed, :side, :last_event
	def initialize(st, ed, side) # side is int from -1 to 1
		@st, @ed = st, ed
		@side = side
		@last_event = nil
	end
	def get_n_d_at(x)
		n = st.y * (ed.x - x) + ed.y * (x - st.x)
		return n, (ed.x - st.x)
	end
	def compare_at_x(x, o)
		n,d = get_n_d_at(x)
		no, d_o = o.get_n_d_at(x)
		n * d_o <=> d * no
	end
	def compare_to_xy(x, pt)
		n,d = get_n_d_at(x)
		n <=> d * pt.y
	end
	def wrap_up(ed_pt)
		@last_event.connect_to(ed_pt, @side)
	end
end

class Status
	def initialize()
		@active = []
	end
	def add_lines(evts, to_adds)
		st = to_adds[0].st
		x = st.x
		i = 0
		i += 1 while (i < @active.length && @active[i].compare_to_xy(x, st) < 0)
		imin = i
		to_adds.each do |to_add|
			@active.insert(i, to_add)
		end
		imax = to_adds.length + imin - 1
		check_pair(evts, imin - 1) if (imin > 0)
		check_pair(evts, imax) if (imax < @active.length - 1)
	end
	def check_pair(evts, ival)
		if (@active[ival].ed != @active[ival + 1].ed && @active[ival].st != @active[ival + 1].st)
			if (inter = intersect(@active[ival], @active[ival + 1]))
				evts.add_event(InterEvent.make(inter, @active[ival], @active[ival + 1]))
			end
		end
	end
	def del_line(evts, to_del)
		i = 0
		i += 1 while (i < @active.length && @active[i] != to_del)
#		raise "error" if i == @active.length
		@active.delete_at(i)
		return if (i <= 0 || i >= @active.length)
		check_pair(evts, i - 1)
	end
	def inter_lines(evts, line1, line2)
		i = 0
		i += 1 while (i < @active.length && @active[i] != line1)
		return false if i == @active.length
		return false if @active[i] != line1 || @active[i + 1] != line2
		@active[i], @active[i + 1] = @active[i + 1], @active[i]
		check_pair(evts, i - 1) if (i > 0)
		check_pair(evts, i + 1) if (i + 2 < @active.length)
		return true
	end
	def get_regions(x, yst, yed)
		#@active.sort! { |a, b|
		#	a.compare_at_x(x, b)
		#}
		y_cur = yst
		regions = []
		cnt = 0
		@active.each do |active|
			n,d = active.get_n_d_at(x)
			if (d != 0)
				y_val = n * d**-1
			else
				y_val = active.st.y
			end
#			puts "yval: #{y_val}"
			regions << [y_cur, y_val, cnt]
			cnt += active.side
			y_cur = y_val
		end
		regions << [y_cur, yed, cnt]
	end
end



class PointInfo
	class TwinnedLine
		attr_accessor :twin, :st, :ed
		def initialize(st, ed, side) @st, @ed, @side = st, ed, side end
		def self.make(st, ed, side)
			ln = self.new(st, ed, side) 
			ltwin = self.new(ed, st, -side)
			ln.twin = ltwin
			ltwin.twin = ln
			return ln
		end
		def to_seg()
			return Shape::Segment.new(@st.pt, @ed.pt) 
		end
		def rot()
			@st.rot(self)
		end
		def next()
			@twin.rot()
		end
	end
	def pt() @pt end
	def initialize(pt)
		raise "problem" if pt.class == Integer
		@pt = pt
		@cons = []
		@ordered = nil
	end
	def do_dedup()
		@cons.uniq!()
	end
	def compare_rot_order(a, b)
		p1 = x_lex_order(a.ed.pt, @pt) <=> x_lex_order(b.ed.pt, @pt)
		return p1 if p1 != 0
		return (a.ed.pt - @pt).cross(b.ed.pt - @pt) <=> 0
	end
	def rot_order()
		@ordered ||= ( 
		@cons.sort! { |a, b|
			compare_rot_order(a, b)
		};
		)
	end
	def rot(item)
		rot_order()
		@cons[(@cons.index(item) - 1) % @cons.length]
	end
	def connect_to(ept, side)
		ln = TwinnedLine.make(self, ept, side)
		insert_ln(ln)
		ept.insert_ln(ln.twin)
	end
	def insert_ln(ln)
		@cons << ln
	end
	# assuming pt is the lex min, get an outside line.
	def lex_min_line()
		rot_order()
		@cons[0]
	end
end

class AddDelEvent
	def initialize(pt, to_add, to_del)
		@pt = pt
		@to_add = to_add
		@to_del = to_del
		@to_add.sort! { |a, b|
			(a.ed - @pt).cross(b.ed - @pt) <=> 0
		}
		@point_info = PointInfo.new(@pt)
	end
	def <=>(o) x_lex_order(pt, o.pt) end
	def pt() @pt end
	def inspect()
		n = "<#{pt.x}, #{pt.y}"
		n += ", a:#{@to_add.length}" if @to_add.length > 0
		n += ", d:#{@to_del.length}" if @to_del.length > 0
		return n + ">"
	end
	def do_event(status, evts)
		@to_del.each do |to_del|
			status.del_line(evts, to_del)
		end
		status.add_lines(evts, @to_add) if @to_add.length > 0
		@to_del.each do |to_del|
			to_del.wrap_up(@point_info)
		end
		@to_add.each do |to_add|
			to_add.last_event = @point_info
		end
	end
	def lex_min_line() @point_info.lex_min_line() end
	def to_del() @to_del.dup() end
	def to_add() @to_add.dup() end
	def merge_in(other)
		@to_del += other.to_del()
		@to_add += other.to_add()
		@to_add.uniq()
		@to_del.uniq()
		@to_add.sort! { |a, b|
			(a.ed - @pt).cross(b.ed - @pt) <=> 0
		}
	end
end

class InterEvent
	def self.make(pt, item1, item2)
		its = [item1, item2]
		AddDelEvent.new(pt, its, its.dup())
	end
	def initialize(pt, item1, item2)
		@pt = pt
		@item1, @item2 = item1, item2
		@point_info = PointInfo.new(@pt)
	end
	def inspect()
		n = "<#{pt.x}, #{pt.y}"
		return n + ">"
	end
	def <=>(o) x_lex_order(pt, o.pt) end
	def pt() @pt end
	def do_event(status, evts)
		if status.inter_lines(evts, @item1, @item2)
			@item1.wrap_up(@point_info)
			@item1.last_event = @point_info 
			@item2.wrap_up(@point_info)
			@item2.last_event = @point_info 
		end
	end
end

class AddEvent
	def initialize(to_add)
		@to_add = to_add
	end
	def <=>(o) x_lex_order(pt, o.pt) end
	def pt() @to_add.st() end
	def inspect() "a<#{pt.x}, #{pt.y}>" end
	def do_event(status)
		status.add_line(@to_add)
	end
end

class DelEvent
	def initialize(to_del)
		@to_del = to_del
	end
	def <=>(o) x_lex_order(pt, o.pt) end
	def pt() @to_del.ed() end
	def inspect() "d<#{pt.x}, #{pt.y}>" end
	def do_event(status)
		status.del_line(@to_del)
	end
end

class LabeledLineList
	def initialize()
		@labs = []
	end
	attr_accessor :labs
	def draw_to(cr, scale)
		@labs.each do |ln|
			st, ed = ln.st.pt, ln.ed.pt
			cr.move_to(scale * st.x, -scale * st.y)
			cr.line_to(scale * ed.x, -scale * ed.y)
			cr.stroke()
		end
	end
	def add_line(ln)
		@labs << ln
	end
	def to_shape()
		Shape.new(@labs.collect { |lab| lab.to_seg() })
	end
end

class ActiveSweepLine
	def initialize(evts)
		@evts = evts
		@status = Status.new()
		@past_evts = []
	end
	def empty()
		@evts.empty()
	end
	def tick()
		min = @evts.pop_min()
		@past_evts << min
		@evts.event_limit = min
		puts "processing event: #{min.inspect}"
		@first_evt ||= min
		@last_x = min.pt.x
		min.do_event(@status, @evts)
	end
	def last_x()
		return @last_x if @evts.empty()
		return @evts.next_event().pt.x
	end
	def get_regions(x, st, ed)
		@status.get_regions(x, st, ed)
	end
	def extract_polys()
		labl = LabeledLineList.new()
		lno = ln = @first_evt.lex_min_line().twin
		i = 0
		while (labl.add_line(ln) ; ln = ln.next(); ln != lno) 
			i += 1
			break if i == 20
		end

#		@first_evt.extract_polys()
		return [labl.to_shape(), @past_evts]
	end
end

class SweepLineUnion
	def self.reset_sweep(shapes)
		evts = EventQueue.new()
		shapes.each do |shape|
			shape.add_evts(evts)
		end
		evts.flip()
		return ActiveSweepLine.new(evts)
	end
	def self.make_outside_profile(shapes)
		sweep_line = reset_sweep(shapes)
		nticks = 0
		while !sweep_line.empty()
			sweep_line.tick()
			nticks += 1
		end
		return sweep_line.extract_polys(), nticks
	end
	def initialize(shapes)
		@shapes = shapes
		@poly, @nticks = self.class.make_outside_profile(shapes)
		@sweep_line = self.class.reset_sweep(@shapes)
		@ticki = 0
	end
	def n_views()
		@nticks
	end
	CntColors = [Color::White, Color::Red, Color::Blue, Color.new(0.5, 0.5, 0.0), Color::Pink, Color.new(0.0,1.0,0.0)]
	def draw_to(cr, scale, view_n)
		if (@ticki > view_n)
			@sweep_line = self.class.reset_sweep(@shapes)
			@ticki = 0
		end
		while (@ticki < view_n)
			@ticki += 1
			@sweep_line.tick()
		end

		@shapes.each { |shape| shape.draw_to(cr, scale) }
		cr.set_source_rgb(0.0, 0.5, 1.0)
		last_x = @sweep_line.last_x
		yed = -cr.get_clip_rectangle[1].y
		yst = yed - cr.get_clip_rectangle[1].height
#		puts cr.get_clip_rectangle[1].y
#		puts cr.get_clip_rectangle[1].height

		yst = ((yst + 10) / scale).to_i
		yed = ((yed - 10) / scale).to_i

		segs = @sweep_line.get_regions(last_x, yst, yed)
		#puts segs.inspect
		segs.each do |y1, y2, cnt|
			CntColors[cnt].set(cr)
#			cr.set_source_rgb(rand(), rand(), rand())
			cr.move_to(last_x * scale, -y1 * scale)
			cr.line_to(last_x * scale, -y2 * scale)
			cr.stroke()
		end

		#Color::.set(cr)
		cr.set_source_rgb(0.0, 1.0, 0.5)
		@poly.draw_to(cr, scale)
	end
end
