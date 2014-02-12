begin
  require "sha1"
rescue LoadError
  require "digest/sha1"
end

class Target::StructBase < Target::Node
	def create_DoGetType(type_class, cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"const ::aos::MessageType*","DoGetType")
		member_func.static = true
		fields = []
    register_members = []
		@members.each do |member|
			tId = member.getTypeID()
			fieldName = member.name.inspect
			if(member.respond_to?(:add_TypeRegister))
        register_members.push(member)
			end
			fields << "new ::aos::MessageType::Field{#{tId}, #{fieldName}}"
		end
    register_members.uniq do |member|
      member.type
    end.each do |member|
			member.add_TypeRegister(cpp_tree, type_class, member_func)
    end
		id = getTypeID()
		member_func.suite << ("static const ::aos::MessageType kMsgMessageType(#{id}, #{@name.inspect}, {" +
		  "#{fields.join(", ")}})");
		type_class.add_member(member_func)
		member_func.suite << "::aos::type_cache::Add(kMsgMessageType)"
		member_func.suite << CPP::Return.new("&kMsgMessageType")
	end
	def simpleStr()
		return "{\n" + @members.collect() { |elem| elem.simpleStr() + "\n"}.join("") + "}"
	end
	def getTypeID()
		return "0x" + (((Digest::SHA1.hexdigest(simpleStr())[0..3].to_i(16)) << 16) + size).to_s(16)
	end
	def add_member(member)
		@members << member
	end
	def size()
		return @size if(@size)
		@size = 0
		@members.each do |elem|
			@size += elem.size
		end
		return @size
	end
end

class Target::MessageDec < Target::StructBase
	attr_accessor :name,:loc,:parent,:msg_hash
	def initialize(name)
		@name = name
		@members = []
	end
	def extern=(value)
		@extern=value
	end
	def get_name()
		if(@parent)
			return "#{@parent.get_name}.#{@name}"
		else
			return "#{@name}"
		end
	end
	def create_Print(type_class,cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"size_t","Print")
		type_class.add_member(member_func)
		member_func.args << "char *buffer"
		member_func.args << "size_t length"
		member_func.const = true
                format = "\""
                args = []
		@members.each do |elem|
			format += ", "
			format += elem.toPrintFormat()
			elem.fetchPrintArgs(args)
		end
    format += "\""
    member_func.suite << "size_t super_size = ::aos::Message::Print(buffer, length)"
    member_func.suite << "buffer += super_size"
    member_func.suite << "length -= super_size"
    member_func.suite << "return super_size + snprintf(buffer, length, " + ([format] + args).join(", ") + ")";
	end
	def create_Serialize(type_class,cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"size_t","Serialize")
		type_class.add_member(member_func)
		#cpp_tree.cc_file.add_funct(member_func)
		member_func.args << "char *buffer"
		member_func.suite << "::aos::Message::Serialize(buffer)"
		member_func.const = true
		offset = 0
		@members.each do |elem|
			elem.toNetwork(offset,member_func.suite)
			offset += elem.size;
		end
                member_func.suite << "return Size()"
	end
	def create_Deserialize(type_class,cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"size_t","Deserialize")
		type_class.add_member(member_func)
		#cpp_tree.cc_file.add_funct(member_func)
		member_func.args << "const char *buffer"
		member_func.suite << "::aos::Message::Deserialize(buffer)"
		offset = 0
		@members.each do |elem|
			elem.toHost(offset,member_func.suite)
			offset += elem.size;
		end
                member_func.suite << "return Size()"
	end
	def create_Zero(type_class,cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"void","Zero")
		type_class.add_member(member_func)
		#cpp_tree.cc_file.add_funct(member_func)
		@members.each do |elem|
			elem.zeroCall(member_func.suite)
		end
		member_func.suite << "::aos::Message::Zero()"
	end
	def create_Size(type_class,cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"size_t","Size")
		member_func.inline = true
        member_func.static = true
		type_class.add_member(member_func.forward_dec)
		#cpp_tree.cc_file.add_funct(member_func)
		size = 0
		@members.each do |elem|
			size += elem.size
		end
		member_func.suite << CPP::Return.new(CPP::Add.new(size,
							"::aos::Message::Size()"))
	end
	def create_GetType(type_class, cpp_tree)
		member_func = CPP::MemberFunc.new(type_class,"const ::aos::MessageType*","GetType")
		type_class.add_member(member_func)
		member_func.static = true
		member_func.suite << "static ::aos::Once<const ::aos::MessageType> getter(#{type_class.name}::DoGetType)"
		member_func.suite << CPP::Return.new("getter.Get()")
	end
	def self.builder_loc(loc)
		return @builder_loc if(@builder_loc)
		return @builder_loc = loc.root.get_make("aos")
	end
	def type_name(builder_loc,name)
		if(builder_loc == @loc) #use relative name
			return name
		else #use full name
			return @loc.to_cpp_id(name)
		end
	end
	def create(cpp_tree)
		return self if(@extern)
		orig_namespace = namespace = cpp_tree.get(@loc)
		name = ""
		if(namespace.class < Types::Type) #is nested
			name = namespace.name + "_" + name
			namespace = namespace.space
		end
		type_class = namespace.add_struct(name + @name)
		if(name.length > 0)
			orig_namespace.add_member(:public, Types::TypeDef.new(type_class,@name))
		end
		cpp_tree.set(self,type_class)
		type_class.set_parent("public ::aos::Message")
		ts = self.simpleStr()
		self.msg_hash = "0x#{Digest::SHA1.hexdigest(ts)[-8..-1]}"
		type_class.add_member("enum {kQueueLength = 1234, kHash = #{self.msg_hash}}")
		@members.each do |elem|
			type_class.add_member(elem.create_usage(cpp_tree))
		end

		create_Serialize(type_class,cpp_tree)
		create_Deserialize(type_class,cpp_tree)
		create_Zero(type_class,cpp_tree)
		create_Size(type_class,cpp_tree)
		create_Print(type_class,cpp_tree)
		create_GetType(type_class, cpp_tree)
		create_DoGetType(type_class, cpp_tree)

		b_namespace = cpp_tree.get(b_loc = self.class.builder_loc(@loc))

		safetemplate = Types::TemplateClass.new(b_namespace,"SafeMessageBuilder")
		ifdef_statement = Types::PreprocessorIf.new(b_namespace,"!defined(__VXWORKS__) && !defined(__TEST_VXWORKS__)")
		ifdef_statement.add_member(safetemplate)
		b_namespace.add(ifdef_statement)
		template = b_namespace.add_template("MessageBuilder")
		safetemplate.spec_args << t = @loc.to_cpp_id(@name)
		template.spec_args << t = @loc.to_cpp_id(@name)
		safemsg_ptr_t = "SafeScopedMessagePtr< #{t}>"
		msg_ptr_t = "ScopedMessagePtr< #{t}>"
		safemsg_bld_t = "SafeMessageBuilder< #{t}>"
		msg_bld_t = "MessageBuilder< #{t}>"
		safetemplate.add_member(:private,"#{safemsg_ptr_t} msg_ptr_")
		template.add_member(:private,"#{msg_ptr_t} msg_ptr_")
		template.add_member(:private,"#{msg_bld_t}(const #{msg_bld_t}&)")
		template.add_member(:private,"void operator=(const #{msg_bld_t}&)")
		safetemplate.add_member(:private,"friend class ::aos::Queue< #{t}>")
		template.add_member(:private,"friend class ::aos::Queue< #{t}>")

		cons = CPP::Constructor.new(template)
		unsafe_cons = CPP::Constructor.new(template)
		cons_ifdef_statement = CPP::PreprocessorIf.new(cons, unsafe_cons)
		cons_ifdef_statement.name = "!defined(__VXWORKS__) && !defined(__TEST_VXWORKS__)"
		template.add_member(:private,cons_ifdef_statement)
		cons.args << "RawQueue *queue"
		cons.args << "#{t} *msg"
		unsafe_cons.args << "#{t} *msg"
		cons.add_cons("msg_ptr_","queue","msg")
		unsafe_cons.add_cons("msg_ptr_","msg")
		cons = safetemplate.add_member(:private,CPP::Constructor.new(safetemplate))
		cons.args << "RawQueue *queue"
		cons.add_cons("msg_ptr_","queue")
		safetemplate.public
		template.public
		DefineMembers(cpp_tree, safetemplate, safemsg_bld_t)
		DefineMembers(cpp_tree, template, msg_bld_t)

	end
	def DefineMembers(cpp_tree, template, msg_bld_t) 
		send = template.def_func("bool","Send")
		send.suite << CPP::Return.new("msg_ptr_.Send()")
		@members.each do |elem|
=begin
			MessageBuilder<frc971::control_loops::Drivetrain::Goal> &steering(
				      double steering) {
				    msg_ptr_->steering = steering;
					    return *this;
						  }
=end
			setter = template.def_func(msg_bld_t,"&#{elem.name}")
			setter.args << elem.create_usage(cpp_tree)
			elem.set_message_builder(setter.suite)
			setter.suite << CPP::Return.new("*this")

		end
	end
end
class Target::MessageElement < Target::Node
	attr_accessor :name,:loc,:size,:zero,:type,:printformat
	def initialize(type,name)
		@type,@name = type,name
	end
	def toPrintFormat()
		return printformat
	end
	def create_usage(cpp_tree)
		"#{@type} #{@name}"
	end
	def toNetwork(offset,suite, parent = "")
		offset = (offset == 0) ? "" : "#{offset} + "
		suite << f_call = CPP::FuncCall.build("to_network",
									 "&#{parent}#{@name}",
									 "&buffer[#{offset}::aos::Message::Size()]")
		f_call.args.dont_wrap = true
	end
	def toHost(offset,suite, parent = "")
		offset = (offset == 0) ? "" : "#{offset} + "
		suite << f_call = CPP::FuncCall.build("to_host",
									 "&buffer[#{offset}::aos::Message::Size()]",
									 "&#{parent}#{@name}")
		f_call.args.dont_wrap = true
	end
	def getTypeID()
		Digest::SHA1.hexdigest(@type)[0..7].to_i(16) | 0x4000 #ensures is primative
	end
	def simpleStr()
		"#{@type} #{@name}"
	end
	def set_message_builder(suite)
		suite << "msg_ptr_->#{@name} = #{@name}"
	end

	def zeroCall(suite, parent = "")
		suite << CPP::Assign.new(parent + @name,@zero)
	end
	def fetchPrintArgs(args, parent = "")
		if (self.type == 'bool')
			args.push("#{parent}#{self.name} ? 'T' : 'f'")
		else
			args.push("#{parent}#{self.name}")
		end
	end
end
class Target::MessageArrayElement < Target::Node
	attr_accessor :name,:loc,:size,:zero,:type
	def initialize(type,name,length)
		@type,@name,@length = type,name,length
	end
	def create_usage(cpp_tree)
		"#{@type} #{@name}[#@length]"
	end
	def type()
		"#{@type}[#@length]"
	end
	def size()
		@size * @length
	end
	def toNetwork(offset,suite)
		offset = (offset == 0) ? "" : "#{offset} + "
		suite << for_stmt = CPP::For.new("int i = 0","i < #{@length}","i++")
		for_stmt.suite << f_call = CPP::FuncCall.build("to_network",
									 "&(#{@name}[i])",
									 "&buffer[i + #{offset}::aos::Message::Size()]")
		f_call.args.dont_wrap = true
	end
	def toHost(offset,suite)
		offset = (offset == 0) ? "" : "#{offset} + "
		suite << for_stmt = CPP::For.new("int i = 0","i < #{@length}","i++")
		for_stmt.suite << f_call = CPP::FuncCall.build("to_host",
									 "&buffer[i + #{offset}::aos::Message::Size()]",
									 "&(#{@name}[i])")
		f_call.args.dont_wrap = true
	end
	def set_message_builder(suite)
		suite << "memcpy(msg_ptr_->#{@name},#{@name},#@length)"
	end
	def zeroCall(suite)
		suite << "memset(#@name,0,#{size()})"
	end
end
