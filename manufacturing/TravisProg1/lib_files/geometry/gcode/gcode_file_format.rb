require "tokens/tokenizer_lib_base.rb"

require_relative "simulate.rb"

class GcodeInterpret
	attr_accessor :stpt
	attr_accessor :spindle_speed, :st_h
	def initialize()
		@cmds = []
	end
	def add_cmd(cmd)
		@cmds << cmd
	end
end
class UnparsedGcode
	def initialize(view)
		@view = view
	end
	def inspect() "<UnparsedGcode#{self.object_id}>" end
end

def expect_view_line(view, exp_line)
	view.st += 1 while view[0].class == CommentLine
	if !view[0].match_gcode(exp_line)
		raise "unexpected: #{exp_line.inspect} vs #{view[0].inspect}"
	end
	view.st += 1
end

def expect_footer_view_line(view, exp_line)
	view.ed -= 1 while view[-1].class == CommentLine
	if !view[-1].match_gcode(exp_line)
		raise "unexpected: #{exp_line.inspect} vs #{view[-1].inspect}"
	end
	view.ed -= 1
end

def expect_not_comment_lines(view, lines)
	lines.split(/\n/).each do |ln|
		expline = parse_gcode_line(ln)
		expect_view_line(view, expline)
	end
end

def expect_footer_not_comment_lines(view, lines)
	lines.split(/\n/).reverse.each do |ln|
		expline = parse_gcode_line(ln)
		expect_footer_view_line(view, expline)
	end
end

def get_spindle_speed(view)
	view.st += 1 while view[0].class == CommentLine
	words = view[0].words
	raise "expected spindle speed, got: #{view[0].inspect}" if (words.length != 2 ||
					words[0].code != "S" || words[1].code != "M" || words[1].number != "3")
	view.st += 1
	return words[0].number
end

def match_header(view)
	expect_not_comment_lines(view, <<END1)
%
G90 G94 G17 G91.1
G64 P0.001 Q0.001
G20
G53 G0 Z0.
M9
T1 M6
END1
# S19638 M3
	speed = get_spindle_speed(view)
expect_not_comment_lines(view, <<END1)
G4 P10.
G54
M7
END1
ln = view[0]
if (ln.words.length != 3 || ln.words[0].code != "G" ||
	  ln.words[0].number != "0" || ln.words[1].code != "X" ||
	  ln.words[2].code != "Y")
	raise "expecting starting point. got #{ln.inspect} "
end
stx = ln.words[1].number
sty = ln.words[2].number
st_pt = Gcode::XYPoint.new(stx.to_f, sty.to_f)

view.st += 1
	expect_not_comment_lines(view, <<END1)
G43 Z0.6 H1
END1
ln = view[0]
if (ln.words.length != 2 || ln.words[0].code != "G" ||
	  ln.words[0].number != "0" || ln.words[1].code != "Z")
	raise "expecting starting height. got #{ln.inspect} "
end
st_h = ln.words[1].number

view.st += 1
#G0 Z#z_height
return st_pt, speed, st_h
end

def match_footer(view)
#	expect_footer_not_comment_lines(view, <<END1)
#M9
#END1
	
expect_footer_not_comment_lines(view, <<END1)
M30
%
END1
ln = view[-1]
if (ln.words.length == 2)
	expline = parse_gcode_line("G53 Z0.")
	expect_footer_view_line(view, expline)
else
	expline = parse_gcode_line("G53 G0 Z0.")
	expect_footer_view_line(view, expline)
end

end

def extract_lines(view, interpret, state)
	view.lines do |ln|
		begin
			if ln.class != CommentLine
				line_cmd, state = state.emit_state_update(ln)
				if (line_cmd)
					interpret.add_cmd(line_cmd)
				end
			end
		rescue Exception => e
			$stderr.puts("Parse problem #{ln.filename}:#{ln.line_number}: #{ln.emit()}")
			raise e
		end
	end
end

def interpret_gcode(gcode_file, do_not_validate = false)
	interpret = GcodeInterpret.new()
	view = GcodeFileView.new(gcode_file)
	st_pt, speed, st_h = match_header(view)
	match_footer(view)

	interpret.stpt = st_pt 
	interpret.st_h = st_h
	interpret.spindle_speed = speed

	if (do_not_validate)
		interpret.add_cmd(UnparsedGcode.new(view))
	else
		state = GcodeState.init_defaults(st_pt)
		extract_lines(view, interpret, state)
	end
	return interpret
end
