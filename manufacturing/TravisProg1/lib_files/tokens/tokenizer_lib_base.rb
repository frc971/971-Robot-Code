class StringBuffer
	def is_buffered(); true;	end
	def initialize(string)
		@string = string
    @line = 1
    @col_nums = []
    @col = 0
		@i = 0
	end
  def lineno()
    return @line
  end
  def filename
    return @filename || "<string>"
  end
  def pos()
    "#{filename()}:#{lineno()},#{@col}"
  end
  def pop_char()
    val = @string[@i,1]
		@i += 1
    @col_nums[@line] = @col += 1
    if(val == "\n")
      @line += 1
      @col = 0
    end
    return val
  end
  def unpop_char(char)
		@i -= 1
    if(char == "\n")
      @line -= 1
      @col = @col_nums[@line]
    end
    @col -= 1
  end
end
class BufferedReader
	def is_buffered(); true;	end
  def initialize(file)
    @file = file
    @line = 1
    @chars = []
    @col_nums = []
    @col = 0
		@i = 0
		@charbuf_i = 1e9
		@ti = 0
  end
	def path()
		@file.path
	end
  def filename
    return File.basename(@file.path)
  end
  def pos()
    "#{filename()}:#{lineno()},#{@col}"
  end
  def clear_comment()
    @file.gets
    @line += 1
  end
  def lineno()
    return @line
    return @file.lineno + 1
  end
	def read_next_char()
		if @charbuf_i >= 2048
			@charbuf = @file.read(2048)
			@ti += 2048
			@charbuf_i = 0
		end
		return nil if @charbuf == nil
		val = @charbuf[@charbuf_i]
		@charbuf_i += 1
		return val
	end
  def pop_char()
		@i += 1
    val = @chars.pop() || read_next_char()
    @col_nums[@line] = @col += 1
    if(val == "\n")
      @line += 1
      @col = 0
    end

    return val
  end
  def unpop_char(char)
		@i -= 1
    if(char == "\n")
      @line -= 1
      @col = @col_nums[@line]
    end
    @col -= 1
    @chars.push(char)
  end
	def get_data()
		ie = @i - @ti + 2048
		is = @old_i - @ti + 2048
		if (is > 0)
			return @charbuf[is...ie]
		end
		#puts "get_data: #{is} #{ie}"
		@file.seek(@old_i, IO::SEEK_SET)
		#@chars = []
		val = @file.read(@i - @old_i)
		@file.seek(@ti, IO::SEEK_SET)
		return val
	end
	def mark_data()
		@old_i = @i - 1
	end
end
require "yaml"
class TokensConvert
	def self.unEscapeString(str)
		return YAML.load(%Q(---\n#{str}\n))
	end
	def self.toFloat(str)
		return str.to_f
	end
	def self.toInteger(str)
		return str.to_i
	end
end
class Module
	def init_fields(*fields)
		fn = "def initialize(#{fields.join(",")})\n"
		fn += "super()\n"
		fields.each do |fname|
			fn += "@#{fname} = #{fname}\n"
		end
		fn += "end\n"
		self.class_eval(fn)
		attr_accessor(*fields)
	end
	def init_fields_ext(*fields)
		fn = "def initialize(#{fields.join(",")})\n"
		fields.each do |fname|
			fn += "@#{fname} = #{fname}\n"
		end
		fn += "initialize_extended()"
		fn += "end\n"
		self.class_eval(fn)
		attr_accessor(*fields)
	end
end
