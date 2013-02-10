class BufferedReader
	def initialize(file)
		@file = file
		@line = 1
		@chars = []
		@col_nums = []
		@col = 0
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
	def pop_char()
		val = @chars.pop() || @file.read(1)
		@col_nums[@line] = @col += 1
		if(val == "\n")
			@line += 1
			@col = 0
		end

		return val
	end
	def unpop_char(char)
		if(char == "\n")
			@line -= 1
			@col = @col_nums[@line]
		end
		@col -= 1
		@chars.push(char)
	end
end
class Tokenizer
	TOKEN_TYPES = {"{" => :tOpenB,"}"=> :tCloseB,";" => :tSemi,"," => :tComma,
		"(" => :tOpenParan,")" => :tCloseParan,"=" => :tAssign,"." => :tDot,
		"<<"=> :tLShift,"*" => :tMult,"+" => :tAdd,"[" => :tOpenB,
		"]" => :tCloseB}
	Humanize = TOKEN_TYPES.invert
	class Token
		attr_accessor :type,:data,:pos
		def to_s
			if(@type == :tString)
				val = @data.inspect.to_s
			elsif(@type == :tWord)
				val = @data.to_s
			else
				val = @data.to_s
			end
			return "#{val.ljust(50)}:#{@type}"
		end
		def humanize()
			if(@type == :tString)
				return "#{@data.inspect.to_s} string"
			elsif(@type == :tWord)
				return "#{@data.inspect} identifier"
			end
			return Humanize[@type].inspect
		end
		def inspect()
			data = ""
			data = " #{@data.inspect}" if(@data)
			"<Token :#{@type}#{data} at #{@pos}>"
		end
		def ==(other)
			if(other.class == Symbol)
				return @type == other
			elsif(other.class == self.class)
				return @type == other.type && @data == other.data
			else
				return nil
			end
		end
	end
	def initialize(file)
		file = File.open(file,"r") if(file.class == String)
		@read = BufferedReader.new(file)
	end
	def qError(error)
		syntax_error(error)
	end
	def syntax_error(msg)
		err = QSyntaxError.new(msg)
		err.qstacktrace << "#{@read.lineno} of #{@read.filename}"
		raise err
	end
	def peak_token()
		@peak_token = next_token()
	end
	def peak()
		peak_token()
	end
	def next()
		next_token()
	end
	def pos()
		@read.pos
	end
	def tokenize(string_token)
		token = Token.new()
		token.type = TOKEN_TYPES[string_token]
		return token
	end
	def next_token()
		if(token = @peak_token)
			@peak_token = nil
			return token
		end
		token = next_token_cache()
		pos = self.pos()
		token.pos = pos
		return token
	end
	def next_token_cache()
		token = Token.new()
		token.data = ""
		while (char = @read.pop_char())
			#puts "#{char.inspect}:#{token.inspect}"
			if(char == "/")
				if(@read.pop_char == "/")
					@read.clear_comment()
				else
					syntax_error("unexpected #{char.inspect}")
				end
			elsif(char == "#")
				@read.clear_comment()
			elsif(char =~ /[\s\r\n]/)
				if(token.type)
					return token
				end
			elsif(char =~ /\"/)
				token.type = :tString
				token.data = ""
				while((char = @read.pop_char) != "\"")
					token.data += char
				end
				return token
			elsif(char =~ /[1-9]/)
				token.type = :tNumber
				token.data = char.to_i
				while(char != ".")
					char = @read.pop_char
					if(char =~ /[0-9]/)
						token.data = char.to_i + token.data * 10
					elsif(char == ".")
					else
						@read.unpop_char(char)
						return token
					end
				end
				second_char = 0
				man = 0
				while(true)
					char = @read.pop_char
					if(char =~ /[0-9]/)
						second_char = char.to_i + second_char * 10
						man = man * 10
					else
						@read.unpop_char(char)
						token.data += second_char / man.to_f
						return token
					end
				end
			elsif(char == ":")
				if(@read.pop_char == "=")
					return tokenize(":=")
				end
				syntax_error("unexpected \":\"")
			elsif(char =~ /[;\{\}=()\",\*\+\.\[\]]/)
				return(tokenize(char))
			elsif(char =~ /[a-zA-Z_]/)
				token.type = :tWord
				token.data = char
				while(true)
					char = @read.pop_char()
					if(char && char =~ /\w/)
						token.data += char
					else
						@read.unpop_char(char)
						return token
					end
				end
			elsif(char == "<")
				if((char = @read.pop_char()) == "<")
					return tokenize("<<")
				else
					@read.unpop_char(char)
					return tokenize("<")
				end
			else
				syntax_error("unexpected #{char.inspect}")
			end
		end
		token.type = :tEnd
		return token
	end
	def expect(type,&blk)
		token = self.next
		if(token != type)
			syntax_error(blk.call(token)) if(blk)
			syntax_error("unexpected: #{token.type}")
		end
		return token
	end
end
