class MessageElementStmt < QStmt
	attr_accessor :name
	def initialize(type_name,name,length = nil) #lengths are for arrays
		@type_name = type_name
		@type = type_name.to_s
    @type = '::aos::time::Time' if @type == 'Time'
		@name = name
		@length = length
	end
	CommonMistakes = {"short" => "int16_t","int" => "int32_t","long" => "int64_t","time" => "Time"}
	def check_type_error(locals)
		if(!(Sizes[@type] || (@length != nil && @type == "char")) )
			if(correction = CommonMistakes[@type])
				raise QError.new(<<ERROR_MSG)
Hey! you have a \"#{@type}\" in your message statement.
\tplease use #{correction} instead. Your type is not supported because we
\twant to guarantee that the sizes of the messages stay the same across platforms.
\tWot. Wot.
ERROR_MSG
			elsif(@type == "char")
				raise QError.new(<<ERROR_MSG)
Hey! you have a \"#{@type}\" in your message statement.
\tyou need your declaration to be a char array like: char[10].
\tor, please use int8_t or uint8_t.
\tWot. Wot.
ERROR_MSG
			else
				@is_struct_type = true
				return if(lookup_type(locals))
				raise QError.new(<<ERROR_MSG)
Hey! you have a \"#{@type}\" in your message statement.
\tThat is not in the list of supported types.
\there is the list of supported types:
\tint{8,16,32,64}_t,uint{8,16,32,64}_t,bool,float,double
\tWot. Wot.
ERROR_MSG
			end
		end
	end

	PrintFormat = {"bool" => "%c",
                       "float" => "%f",
                       "char" => "%c",
                       "double" => "%f",
                       "uint8_t" => "%\" PRIu8 \"",
                       "uint16_t" => "%\" PRIu16 \"",
                       "uint32_t" => "%\" PRIu32 \"",
                       "uint64_t" => "%\" PRIu64 \"",
                       "int8_t" => "%\" PRId8 \"",
                       "int16_t" => "%\" PRId16 \"",
                       "int32_t" => "%\" PRId32 \"",
                       "int64_t" => "%\" PRId64 \"",
                       "::aos::time::Time" => "\" AOS_TIME_FORMAT \""}
        def toPrintFormat()
		if(format = PrintFormat[@type])
			return format;
		end
		raise QError.new(<<ERROR_MSG)
Somehow this slipped past me, but
\tI couldn't find the print format of #{@type}. Really, my bad.
\tWot. Wot.
ERROR_MSG
	end

	Sizes = {"bool" => 1, "float" => 4,"double" => 8,"::aos::time::Time" => 8}
	[8,16,32,64].each do |len|
		Sizes["int#{len}_t"] = len / 8
		Sizes["uint#{len}_t"] = len / 8
	end
	Zero = {"float" => "0.0f","double" => "0.0","bool" => "false","::aos::time::Time" => "::aos::time::Time(0, 0)"}
	def size()
		if(size = Sizes[@type]); return size; end
		return 1 if(@type == "char")
		raise QError.new(<<ERROR_MSG)
Somehow this slipped past me, but
\tI couldn't find the size of #{@type}. Really, my bad.
\tWot. Wot.
ERROR_MSG
	end
	def lookup_type(locals)
		return @type_name.lookup(locals)
	end
	def q_eval(locals)
		check_type_error(locals)
		if(@is_struct_type)
			tval = lookup_type(locals)
			member = Target::MessageStructElement.new(tval, @name)
      if (@length != nil)
        inner_member = member
        member = Target::MessageArrayElement.new(inner_member, @length)
      end
		else
			member = Target::MessageElement.new(@type,@name)
			member.size = size()
			member.zero = Zero[@type] || "0";
			member.printformat = toPrintFormat()

			if(@length != nil)
        inner_member = member
				member = Target::MessageArrayElement.new(inner_member,@length)
			end
		end
		locals.local.add_member(member)
	end
	def self.parse(tokens)
		line = tokens.pos
		#type = tokens.expect(:tWord).data
		type_name = QualifiedName.parse(tokens)
		len = nil
		if(tokens.peak == :tOpenB)
			tokens.expect(:tOpenB)
			len = tokens.expect(:tNumber).data
			tokens.expect(:tCloseB)
		end
		name = tokens.expect(:tWord).data
		tokens.expect(:tSemi)
		return self.new(type_name,name,len).set_line(line)
	end
end
class MessageStmt < QStmt
	def initialize(name,suite)
		@name = name
		@suite = suite
	end
	def q_eval(locals)
		group = Target::MessageDec.new(@name)
		locals.register(group)
		@suite.each do |stmt|
			stmt.q_eval(locals.bind(group))
		end 
		return group
	end
	def self.parse(tokens)
		name = tokens.expect(:tWord).data
		values = []
		tokens.expect(:tOpenB)
		while(tokens.peak != :tCloseB)
			values << MessageElementStmt.parse(tokens)
		end
		names = {}
		values.each do |val|
			if(names[val.name])
				raise QSyntaxError.new(<<ERROR_MSG)
Hey! duplicate name #{val.name.inspect} in your message declaration statement (message #{name}).
\tI found them at: #{names[val.name].q_stack_name()} and #{val.q_stack_name()}.
\tWot. Wot.
ERROR_MSG
			end
			names[val.name] = val
		end
		tokens.expect(:tCloseB)
		tokens.expect(:tSemi)
		self.new(name,values)
	end
end
class QueueStmt < QStmt
	def initialize(type,name)
		@type,@name = type,name
	end
	def q_eval(locals)
		queue = Target::QueueDec.new(@type.lookup(locals),@name)
		locals.register(queue)
		locals.local.add_queue(queue) if(locals.local.respond_to?(:add_queue))
		return queue
	end
	def self.parse(tokens)
		line = tokens.pos
		type_name = QualifiedName.parse(tokens)
		name = tokens.expect(:tWord).data
		tokens.expect(:tSemi)
		return self.new(type_name,name).set_line(line)
	end
end
