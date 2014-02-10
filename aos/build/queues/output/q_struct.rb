class Target::StructDec < Target::Node
	attr_accessor :name,:loc,:parent, :extern
	def initialize(name)
		@name = name
		@members = []
	end
	def add_member(member)
		@members << member
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

		@members.each do |elem|
			type_class.add_member(elem.create_usage(cpp_tree))
		end
		return type_class
	end
	def size()
		return @size if(@size)
		@size = 0
		@members.each do |elem|
			@size += elem.size
		end
		return @size
	end
	def getPrintFormat()
		return "{" + @members.collect { |elem| elem.toPrintFormat() }.join(", ") + "}"
	end
	def fetchPrintArgs(args, parent = "")
		@members.each do |elem|
			elem.fetchPrintArgs(args, parent)
		end
	end
	def toHost(offset, suite, parent)
		@members.each do |elem|
			elem.toHost(offset, suite, parent)
			offset += elem.size()
		end
	end
	def toNetwork(offset, suite, parent)
		@members.each do |elem|
			elem.toNetwork(offset, suite, parent)
			offset += elem.size()
		end
	end
	def zeroCall(suite, parent)
		@members.each do |elem|
			elem.zeroCall(suite, parent)
		end
	end
end
class Target::MessageStructElement < Target::Node
	attr_accessor :name,:loc
	def initialize(type,name)
		@type, @name = type, name
	end
	def type()
		return @type.get_name
	end
	def type_name(cpp_tree)
		type = cpp_tree.get(@type)
		if(@type.loc == @loc) #use relative name
			return type.name
		else #use full name
			return @type.loc.to_cpp_id(type.name)
		end 
	end
	def size()
		return @type.size()
	end
	def toPrintFormat()
		@type.getPrintFormat()
	end
	def create_usage(cpp_tree)
		return "#{type_name(cpp_tree)} #{@name}"
	end
	def fetchPrintArgs(args, parent = "")
		@type.fetchPrintArgs(args, parent + "#{@name}.")
	end
	def toNetwork(offset,suite, parent = "")
		@type.toNetwork(offset, suite, parent + "#{@name}.")
	end
	def toHost(offset,suite, parent = "")
		@type.toHost(offset, suite, parent + "#{@name}.")
	end
	def set_message_builder(suite)
		suite << "msg_ptr_->#{@name} = #{@name}"
	end

	def zeroCall(suite, parent = "")
		@type.zeroCall(suite, parent + "#{@name}.")
	end
	
end
