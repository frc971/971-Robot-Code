

module Gcode
	class CmdLineView
		def initialize(line)
			@word_mapping = {}
			line.words.each do |word|
				(@word_mapping[word.code] ||= []) << word.number
			end
			#puts @word_mapping.inspect
		end
		def del_code(code)
			l = @word_mapping.delete(code)
			return l if !l
			raise "expected less than one word of type: #{code}" if (l.length > 1)
			return l[0]
		end
		def del_code_list(code)
			l = @word_mapping.delete(code) || []
			return l
		end
		def get(code)
			return @word_mapping[code]
		end
		def assert_empty()
			raise "word list not empty #{@word_mapping}" if @word_mapping.length != 0
		end
		def format_as_gcode()
			return @word_mapping.collect do |k, v|
				v.collect {|a| "#{k}#{a}"}.join(" ")
			end.join(" ")
		end
	end
	class XYPoint
		init_fields :x, :y
		def +(o) XYPoint.new(@x + o.x, @y + o.y) end
		def -(o) XYPoint.new(@x - o.x, @y - o.y) end
		def /(o) XYPoint.new(@x / o, @y / o) end
		def *(o) XYPoint.new(@x * o, @y * o) end
		def mag_sqr() return @x * @x + @y * @y end
		def dot(o) @x * o.x + @y * o.y end
		def cross(o) @x * o.y - @y * o.x end
		def ==(o) self.class == o.class && @x == o.x && @y == o.y end
	end
	class XYZPoint
		init_fields :x, :y, :z
		def -@() XYZPoint.new(-@x, -@y, -@z) end
		def ==(o) o.class == XYZPoint && @x == o.x && @y == o.y && @z == o.z && @x && @y && @z end
	end
	class ArcPlane
		attr_accessor :twin
		init_fields :vect
		def self.make(pt)
			a = ArcPlane.new(pt)
			b = ArcPlane.new(-pt)
			a.twin = b
			b.twin = a
			return a
		end
		def has_x() @vect.x == 0 end
		def has_y() @vect.y == 0 end
		def has_z() @vect.z == 0 end
	end
	XYPlane = ArcPlane.make(XYZPoint.new(0, 0, 1)) 
	XZPlane = ArcPlane.make(XYZPoint.new(0, 1, 0)) 
	YZPlane = ArcPlane.make(XYZPoint.new(1, 0, 0)) 
	class Motion
	end
	class Arc < Motion
	end
	class CCWArc < Arc
		init_fields :c, :st, :ed, :arc_plane, :mach_state
		def self.state_construct(s1, s2)
			self.new(s2.arc_center, s1.cur_pt, s2.cur_pt, s2.arc_plane, s2.mach_state())
		end
	end
	class CWArc < Arc
		init_fields :c, :st, :ed, :arc_plane, :mach_state
		def self.state_construct(s1, s2)
			self.new(s2.arc_center, s1.cur_pt, s2.cur_pt, s2.arc_plane, s2.mach_state())
		end
	end
	class Rapid < Motion
		init_fields :st, :ed, :arc_plane, :mach_state
		def self.state_construct(s1, s2)
			self.new(s1.cur_pt, s2.cur_pt, s2.arc_plane, s2.mach_state())
		end
	end
	class Linear < Motion
		init_fields :st, :ed, :arc_plane, :mach_state
		def self.state_construct(s1, s2)
			self.new(s1.cur_pt, s2.cur_pt, s2.arc_plane, s2.mach_state())
		end
	end
end

class GcodeState
	class MachineState
		init_fields :feed_rate, :mist, :flood, :cutter_diameter, :cutter_comp
		def self.init_defaults()
			return self.new(nil, nil, nil, nil, nil)
		end
		def update_mach_state(view)
			feed_rate = view.del_code("F") || @feed_rate
			cutter_diameter = view.del_code("D") || @cutter_diameter
			mist, flood = @mist, @flood
			view.del_code_list("M").each do |code|
				if (code == "7")
					mist = :on
				elsif (code == "8")
					flood = :on
				elsif (code == "9")
					flood = :off
					mist = :off
				else
					raise "unknown code: #{code}"
				end
			end
			MachineState.new(feed_rate, mist, flood, cutter_diameter, @cutter_comp)
		end
    def set_cutter_comp(new_comp)
			MachineState.new(@feed_rate, @mist, @flood, @cutter_diameter, new_comp)
    end
	end
	attr_accessor :cur_pt, :arc_plane, :arc_center, :mach_state, :motion_type
	def initialize(cur_pt, arc_plane, arc_center, motion_type, mach_state)
		@cur_pt = cur_pt
		@arc_plane = arc_plane
		@arc_center = arc_center
		@motion_type = motion_type
		@mach_state = mach_state
	end
	def mach_state() @mach_state end
	def self.init_defaults(st_pt)
		z = st_pt.respond_to?(:z) ? st_pt.z : nil
		st_pt = Gcode::XYZPoint.new(st_pt.x, st_pt.y, z)
		cent = Gcode::XYZPoint.new(nil, nil, nil)
		mach_state = MachineState.init_defaults()
		return self.new(st_pt, Gcode::XYPlane, cent, nil, mach_state)
	end
end

class G98ModalMotion
	def initialize(view)
		@view = view
	end
	def emit_gcode(f, state)
		f.puts "#{@view.format_as_gcode}"
		return state
	end
end

class G80ModalMotion
	def initialize()
	end
	def emit_gcode(f, state)
		f.puts "G80"
		state.motion_type = nil
		return state
	end
end

class ExpectClearModalMotion
	def initialize(pstate)
		@pstate = pstate
	end
	def emit_state_update(gcode_line)
		view = Gcode::CmdLineView.new(gcode_line)
		init_g_codes = view.del_code_list("G")
		if (init_g_codes.empty?())
			return G98ModalMotion.new(view), self
		end
		if (init_g_codes != ["80"])
			raise "G98 supposed to be followed by G80"
		end
		view.assert_empty()
		return G80ModalMotion.new(), @pstate
	end
end

class GcodeState
	def pt_to_f(v)
		return nil if !v
		raise "not a float: #{v}" if (!v =~ /-?[0-9]+\.[0-9]*/)
		return v.to_f
	end
	def get_npoint(gcode_line)
		xval = pt_to_f(gcode_line.del_code("X"))
		yval = pt_to_f(gcode_line.del_code("Y"))
		zval = pt_to_f(gcode_line.del_code("Z"))
		return Gcode::XYZPoint.new(xval ? xval : @cur_pt.x, 
															 yval ? yval : @cur_pt.y, 
															 zval ? zval : @cur_pt.z) 
	end
	def get_center(gcode_line)
		xval = pt_to_f(gcode_line.del_code("I"))
		yval = pt_to_f(gcode_line.del_code("J"))
		zval = pt_to_f(gcode_line.del_code("K"))
		return Gcode::XYZPoint.new(xval ? xval + @cur_pt.x : @arc_center.x, 
															 yval ? yval + @cur_pt.y : @arc_center.y, 
															 zval ? zval + @cur_pt.z : @arc_center.z) 
	end
	def get_arc_plane()
		return @arc_plane
	end
	def construct_update_cmd(old_state)
		if (old_state.cur_pt == self.cur_pt)
			return nil
		else
			return @motion_type.state_construct(old_state, self)
		end
	end
	def emit_state_update(gcode_line)
		view = Gcode::CmdLineView.new(gcode_line)
		init_g_code = (view.get("G") || [])[0]
		if (init_g_code == "98") || (init_g_code == "83")
			return G98ModalMotion.new(view), ExpectClearModalMotion.new(self)
		end
		npoint = get_npoint(view)
		center = get_center(view)
		mach_state = @mach_state.update_mach_state(view)
		codes = view.del_code_list("G")
		arc_plane = @arc_plane
		motion_type = @motion_type
		codes.each do |code|
			if (code == "0")
				motion_type = Gcode::Rapid
			elsif (code == "1")
				motion_type = Gcode::Linear
			elsif (code == "2")
				motion_type = Gcode::CWArc
			elsif (code == "3")
				motion_type = Gcode::CCWArc
			elsif (code == "40")
        mach_state = mach_state.set_cutter_comp(nil)
			elsif (code == "41")
        mach_state = mach_state.set_cutter_comp(:left)
			elsif (code == "42")
        mach_state = mach_state.set_cutter_comp(:right)
			elsif (code == "17")
				arc_plane = Gcode::XYPlane 
			elsif (code == "18")
				arc_plane = Gcode::XZPlane 
			elsif (code == "19")
				arc_plane = Gcode::YZPlane 
			else
				raise "unknown code: #{code}"
			end
		end
		view.assert_empty()
		nstate = self.class.new(npoint, arc_plane, center, motion_type, mach_state)
		updated_cmd = nstate.construct_update_cmd(self)
		return updated_cmd, nstate
	end
end
