class GcodeLine
	attr_accessor :words, :line_number, :filename
	def initialize(words)
		@words = words
	end
	def match_gcode(o)
		return false if self.class != o.class
		return false if @words.length != o.words.length
		@words.length.times do |i|
			return false if @words[i] != o.words[i]
		end
		return true
	end
	def emit() "#{@words.collect {|word| word.emit()}.join(" ") }" end
end

class GcodePercentLine
	attr_accessor :line_number, :filename
	def match_gcode(o) return self.class == o.class end
end

class CommentLine
	attr_accessor :line_number, :filename
	attr_accessor :cmt
	def initialize(cmt) @cmt = cmt end
end

class GcodeWord
	attr_accessor :code, :number
	def inspect() "#{@code.inspect}#{@number.inspect}" end
	def initialize(code, number)
		@code, @number = code, number
	end
	def emit() "#{@code}#{@number}" end
	def ==(o)
		@code == o.code && @number == o.number
	end
end

class GcodeFile
	attr_accessor :lines
	def initialize()
		@lines = []
	end
	def add_line(ln) @lines << ln end
	def [](i) return @lines[i] end
	def gcode_file() self end
	def st() 0 end
	def ed() @lines.length end
end

class GcodeFileView
	def initialize(gcodef)
		@gcodef = gcodef.gcode_file()
		@st = gcodef.st()
		@ed = gcodef.ed()
	end
	def st() @st end
	def st=(v) @st = v end
	def ed() @ed end
	def ed=(v) @ed = v end
	def gcode_file() @gcodef end
	def length() @ed - @st end
	def [](i)
		l = length()
		return nil if (i < -l || i >= l)
		return @gcodef[i >= 0 ? i + @st : @ed + i]
	end
	def lines(&blk)
		return Enumerator.new do |y|
			(@st...@ed).each do |i|
				y << @gcodef[i]
			end
		end if !blk
		lines.each(&blk)
		self
	end
end

def parse_gcode_line(line)
	line_no_space = line.gsub(/\s/,"")
	if line_no_space[0] == "(" && line_no_space[-1] == ")"
		#puts "comment_line: #{line.inspect}"
		return CommentLine.new(line.chomp)
	elsif (line_no_space == "%")
		#puts "percent_line: \"%\""
		return GcodePercentLine.new()
	else
		wl = line_no_space.scan(/[A-Za-z][0-9.\-]*/)
		raise "Invalid line: #{line_no_space}"if (wl.join() != line_no_space)
		#puts wl.inspect + " vs " + line.inspect
		return GcodeLine.new(wl.collect { |w| GcodeWord.new(w[0], w[1..-1])})
	end
end

def load_gcode(fname)
	i = 0
	fout = GcodeFile.new()
  File.readlines(fname).each do |line|
		line = parse_gcode_line(line) 
		i += 1
		if (line)
			line.line_number = i
			line.filename = fname
			fout.add_line(line)
		end
	end
	return fout
end
