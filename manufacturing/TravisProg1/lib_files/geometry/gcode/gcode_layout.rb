require_relative "simulate.rb"
require_relative "../point2d.rb"

module Gcode
	class Motion
		def emit_gcode(f, state)
			state = state.change_arc_plane(f, @arc_plane)
			state, cmd_tag = state.change_mach_state(f, @mach_state)
			f.puts emit_gcode_cmd(f, state) + cmd_tag
			state.motion_type = self.class
			return state
		end
	end
	class XYZPoint
		def to_s()
			"<#{@x.inspect}, #{@y.inspect}, #{@z.inspect}>"
		end
	end
	class Arc < Motion
		def emit_gcode(f, state)
			state = state.change_arc_plane(f, @arc_plane)
			state, cmd_tag = state.change_mach_state(f, @mach_state)
			f.puts emit_gcode_cmd(f, state) + cmd_tag
			state.motion_type = self.class
			return state
		end
	end
	class CCWArc < Arc
		def emit_gcode_cmd(f, state)
			cmd = ""
			cmd += "G3 " if state.motion_type != self.class
			state.assert_cur_pt(@st)
			da = state.update_arc_center(@c)
			cmd += state.update_pos(@ed)
			cmd += da
		end
		def inspect
			"CCWArc #{@st} -> #{@ed} c:#{@c}"
		end
	end
	class CWArc < Arc
		def emit_gcode_cmd(f, state)
			cmd = ""
			cmd += "G2 " if state.motion_type != self.class
			state.assert_cur_pt(@st)
			da = state.update_arc_center(@c)
			cmd += state.update_pos(@ed)
			cmd += da
		end
		def inspect
			"CWArc #{@st} -> #{@ed} c:#{@c}"
		end
	end
	class Rapid < Motion
		def emit_gcode_cmd(f, state)
			cmd = ""
			cmd += "G0 " if state.motion_type != self.class
			state.assert_cur_pt(@st)
			cmd += state.update_pos(@ed)
		end
		def inspect
			"Rapid #{@st} -> #{@ed}"
		end
	end
	class Linear < Motion
		def emit_gcode_cmd(f, state)
			cmd = ""
			cmd += "G1 " if state.motion_type != self.class
			state.assert_cur_pt(@st)
			cmd += state.update_pos(@ed)
		end
		def inspect
			"Linear #{@st} -> #{@ed}"
		end
	end
end

class GcodeState
	class MachineState
		def change_mach_state(f, o)
			cmd_tag = ""
			if @feed_rate != o.feed_rate && o.feed_rate
				cmd_tag = "F#{o.feed_rate} "
				@feed_rate = o.feed_rate
			end
			tmist = o.mist || @mist
			tflood = o.flood || @flood
			if (tmist == :off && @mist != :off) || (tflood == :off && @flood != :off)
				f.puts("M9")
				@mist = :off
				@flood = :off
			end
			if tmist == :on && @mist == :off
				f.puts("M7")
				@mist = :on
			end
			if tflood == :on && @flood == :off
				f.puts("M8")
				@flood = :on
			end
      if o.cutter_comp != @cutter_comp
        if o.cutter_comp == nil
          f.puts("G40")
        elsif o.cutter_comp == :right || o.cutter_comp == :left
          code = "2"
          code = "1" if o.cutter_comp == :left
          if o.cutter_diameter == nil
            throw Exception.new("missing cutter diameter")
          end
          f.puts("G4#{code} D#{o.cutter_diameter}")
          @cutter_diameter = o.cutter_diameter
        else
          throw Exception.new("invalid_cutter_comp #{o.cutter_comp}")
        end
        @cutter_comp = o.cutter_comp
      end
      if @cutter_diameter != o.cutter_diameter
        if o.cutter_diameter != nil
          f.puts("D#{o.cutter_diameter}")
        end
        @cutter_diameter = o.cutter_diameter
      end
			return cmd_tag
		end
	end
	def comp_d(d1, d2)
		return d2 && (d1 != d2)
	end
	def assert_cur_pt(pt)
		puts pt.inspect
		if comp_d(pt.x, @cur_pt.x) || 
			 comp_d(pt.y, @cur_pt.y) ||
			 comp_d(pt.z, @cur_pt.z)
			raise "problem! <#{pt.x}, #{pt.y}, #{pt.z}> " + 
			    "vs cur:<#{@cur_pt.x}, #{@cur_pt.y}, #{@cur_pt.z}>"
		end
	end
	def fmt_float(f)
		"%f"%f
	end
	def update_pos(pt)
		cmd = ""
		cmd += "X#{fmt_float(pt.x)} " if (pt.x && @cur_pt.x != pt.x)
		cmd += "Y#{fmt_float(pt.y)} " if (pt.y && @cur_pt.y != pt.y)
		cmd += "Z#{fmt_float(pt.z)} " if (pt.z && @cur_pt.z != pt.z)
		@cur_pt.x = pt.x || @cur_pt.x
		@cur_pt.y = pt.y || @cur_pt.y
		@cur_pt.z = pt.z || @cur_pt.z
		return cmd
	end
	def update_arc_center(pt)
		cmd = ""
		ap = @arc_plane
		cmd += "I#{fmt_float(pt.x - @cur_pt.x)} " if (pt.x && ap.has_x)# != @arc_center.x && pt.x)
		cmd += "J#{fmt_float(pt.y - @cur_pt.y)} " if (pt.y && ap.has_y)# != @arc_center.y && pt.y)
		cmd += "K#{fmt_float(pt.z - @cur_pt.z)} " if (pt.z && ap.has_z)# != @arc_center.z && pt.z)
		@arc_center.x = pt.x || @arc_center.x
		@arc_center.y = pt.y || @arc_center.y
		@arc_center.z = pt.z || @arc_center.z
		return cmd
	end
	def change_arc_plane(f, arc_plane)
		return self if (arc_plane == @arc_plane)
		if (Gcode::XYPlane == arc_plane)
			f.puts("G17")
		elsif (Gcode::XZPlane == arc_plane)
			f.puts("G18")
		elsif (Gcode::YZPlane == arc_plane)
			f.puts("G19")
		end
		@arc_plane = arc_plane
		return self
	end
	def change_mach_state(f, mach_state)
		cmd_tag = @mach_state.change_mach_state(f, mach_state)
		return self, cmd_tag
	end
end

class UnparsedGcode
	def emit_gcode(f, state)
		@view.lines do |ln|
			if (ln.class != CommentLine)
				f.puts(ln.emit())
			end
		end
		return state
	end
end

class GcodeInterpret
	def emit_gcode(f, state)
		f.puts(<<END)
S#{@spindle_speed} M3
G0 X#{@stpt.x} Y#{@stpt.y}
G43 Z0.6 H1
G0 Z#{@st_h}
END
		state.cur_pt.x = @stpt.x
		state.cur_pt.y = @stpt.y
		state.cur_pt.z = nil
		@cmds.each do |cmd|
			puts cmd.inspect
			state = cmd.emit_gcode(f, state)
		end
		return state
	end
end

class GcodeLayout
	def initialize()
		@items = []
		@start
	end
	class Pos
		def initialize(gcode, pos, rot)
			@gcode = gcode
			@pos = pos
			@rot = rot
		end
		def spindle_speed() @gcode.spindle_speed end
		def emit_gcode(f, state)
			f.puts(<<END)
G10 L2 P9 X[#5221+#{@pos.x}] Y[#5222+#{@pos.y}] Z[#5223] R#{@rot * 180.0 / Math::PI}
G59.3
END
			
			return @gcode.emit_gcode(f, state)
		end
	end

	def add_item(gcode, pos, rot)
		@items << Pos.new(gcode, pos, rot)
	end

	def write_gcode(fname)
		state = GcodeState.init_defaults(Gcode::XYZPoint.new(0,0,0))
		File.open(fname, "w+") { |f|
		f.puts(<<END)
%
G90 G94 G17 G91.1
G64 P0.001 Q0.001
G20
G53 G0 Z0.
(BORE8)
M9
T1 M6
S#{@items[0].spindle_speed} M3
G4 P10.
G54
M7
END
@items.each do |item|
	state = item.emit_gcode(f, state)
end
		f.puts(<<END)
M9
G53 G0. Z0.
M30
END
		}
	end
end
