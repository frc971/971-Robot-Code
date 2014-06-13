class Target::QueueDec < Target::Node
	attr_accessor :name,:loc
	def initialize(type,name)
		@type,@name = type,name
	end
	def msg_hash()
		return @type.msg_hash
	end
	def full_message_name(cpp_tree)
		type = cpp_tree.get(@type)
		return @type.loc.to_cpp_id(type.name)
	end
	def full_builder_name(cpp_tree)
		return "::aos::MessageBuilder< #{full_message_name(cpp_tree)}>"
	end
	def full_type_name(cpp_tree)
		return "::aos::Queue< #{full_message_name(cpp_tree)}>"
	end
	def type_name(cpp_tree,queue_type = "::aos::Queue")
		type = cpp_tree.get(@type)
		if(@type.loc == @loc) #use relative name
			return "#{queue_type}<#{type.name}>"
		else #use full name
			return "#{queue_type}< #{@type.loc.to_cpp_id(type.name)}>"
		end
	end
	def create_usage(cpp_tree)
		"#{type_name(cpp_tree)} #{@name}"
	end
	def create(cpp_tree)
		namespace = cpp_tree.get(@loc)
		type = cpp_tree.get(@type)
		cpp_tree.set(self,self)
		type_name = type_name(cpp_tree)
		full_type_name = full_type_name(cpp_tree)
		namespace.add_cc("static #{type_name} *#{@name}_ptr")
		counter = "#{@name}_counter"
		namespace.add_cc("static int #{counter}")

		str = <<COMMENT_END
Schwarz counter to construct and destruct the #{@name} queue.
Must be constructed before &#{@name} is constructed so that #{@name}_ptr
is valid so we can store a reference to the object.
COMMENT_END
		str.split(/\n/).each do |str_sec|
			namespace.class_comment(str_sec)
		end
		init_class = namespace.add_class("InitializerFor_" + @name)

		init_class.add_cc_comment(
        "The counter is initialized at load-time, i.e., before any of the static",
		"objects are initialized.")


		cons = CPP::Constructor.new(init_class)
		init_class.add_member(:public,cons)
		cons.suite << if_stmt = CPP::If.new("0 == #{counter}++")

		cons_call = CPP::FuncCall.new("new #{type_name}")
		cons_call.args.push(@loc.queue_name(@name).inspect)
		if_stmt.suite << CPP::Assign.new("#{@name}_ptr",cons_call)


		destruct = CPP::Destructor.new(init_class)
		init_class.add_member(:public,destruct)
		destruct.suite << if_stmt = CPP::If.new("0 == --#{counter}")
		if_stmt.suite << "delete #{@name}_ptr"
		if_stmt.suite << CPP::Assign.new("#{@name}_ptr","NULL")

		init_class.static = true
		init_class.dec = @name + "_initializer";

		str = <<COMMENT_END
Create a reference to the new object in the pointer.  Since we have already
created the initializer
COMMENT_END
		str.split(/\n/).map{|str_sec| CPP::Comment.new(str_sec)}.each do |comment|
      namespace.add(comment)
    end
		namespace.add("static UNUSED_VARIABLE #{type_name} &#{@name}" +
                  " = #{@name}_initializer.get()")

		get = init_class.def_func(full_type_name,"get")
		get.pre_func_types = "&"
		get.suite << CPP::Return.new("*#{@name}_ptr")
		
	end
end

