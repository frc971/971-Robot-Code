require "sha1"
module Target
end
class Target::Node
	attr_accessor :created_by
end
class Target::QFile < Target::Node
	def initialize() #needs to know repo_path, 
		@class_types = []
	end
	def add_type(type)
		@class_types << type
	end
	def extern=(value)
		@class_types.each do |type|
			type.extern=value
		end
	end
	def make_cpp_tree(rel_path)
		cpp_tree = DepFilePair.new(rel_path)
		cpp_tree.add_header_include("\"aos/common/macros.h\"")
		cpp_tree.add_header_include("\"aos/common/queue.h\"")
		@class_types.each do |type|
			cpp_tree.get(type)
		end
		return cpp_tree
	end
end
class Target::QInclude < Target::Node
	def initialize(path)
		@path = path
	end
	def create(cpp_tree)
#		inc = cpp_tree.header.add_include("\"#{@path}\"")
		inc = cpp_tree.add_header_include("\"#{@path}\"")
		cpp_tree.set(self,inc)
	end
end
class Target::QueueGroupDec < Target::Node
	attr_accessor :name,:loc,:parent,:queues
	def initialize(name)
		@name = name
		@queues = []
		@subs = {}
	end
	def root()
		@parent.root()
	end
	def []=(key,val)
		#puts "#{key}= #{val}"
		@subs[key] = val
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
	def to_cpp_id(id)
		name = "#{@name}::#{id}"
		return @parent.to_cpp_id(name) if(@parent)
		return name
	end
	def [](key)
		#puts "#{key}"
		@subs[key]
	end
	def add_queue(queue)
		@queues.push(queue)
	end
	def hash_with_name(name)
		ts = (@queues.collect { |queue|
			queue.msg_hash()
		}).join("") + name
		return "0x#{SHA1.hexdigest(ts)[-8..-1]}"
	end
	def create(cpp_tree)
		return if(@extern)
		namespace = cpp_tree.get(@loc)
		type_class = Types::Class.new(namespace,@name)
		cpp_tree.set(self,type_class)  #breaks infinite recursion
		type_class.set_parent("public ::aos::QueueGroup")
		@queues.each do |queue|
			type_class.add_member(:public,queue.create_usage(cpp_tree))
			namespace.add_pre_swig("%immutable #{@name}::#{queue.name}")
		end
		create_Constructor(type_class,cpp_tree)
		namespace.add(type_class)
	end
	def create_Constructor(type_class,cpp_tree)
		member_func = CPP::Constructor.new(type_class)
		type_class.add_member(:public,member_func)
		#cpp_tree.cc_file.add_funct(member_func)
		member_func.args << "const char *name";
		member_func.args << "uint32_t hash";
		member_func.add_cons("::aos::QueueGroup","name", "hash")
		@queues.each do |queue|
			member_func.args << "const char *#{queue.name}_name";
			member_func.add_cons(queue.name,"#{queue.name}_name")
		end
	end
end
class Target::QueueGroup < Target::Node
	attr_accessor :name,:loc
	def initialize(type,name)
		@type,@name = type,name
	end
	def type_name()
		if(@type.loc == @loc) #use relative name
			return @type.name
		else #use full name
			return @type.loc.to_cpp_id(@type.name)
		end 
	end
	def create(cpp_tree)
		namespace = cpp_tree.get(@loc)
		type = cpp_tree.get(@type)
		cpp_tree.set(self,self)
		namespace.add_cc("static #{type_name} *#{@name}_ptr")
		counter = "#{@name}_counter"
		namespace.add_cc("static int #{counter}")

		str = <<COMMENT_END
Schwarz counter to construct and destruct the #{@name} object.
Must be constructed before &#{@name} is constructed so that #{@name}_ptr
is valid so we can store a reference to the object.
COMMENT_END
		str.split(/\n/).each do |str_sec|
			namespace.class_comment(str_sec)
		end
		init_class = namespace.add_class("InitializerFor_" + @name).add_dep(type)
		

		init_class.add_cc_comment(
        "The counter is initialized at load-time, i.e., before any of the static",
		"objects are initialized.")


		cons = CPP::Constructor.new(init_class)
		init_class.add_member(:public,cons)
		cons.suite << if_stmt = CPP::If.new("0 == #{counter}++")
		if_stmt.suite << "printf(#{"making a #{@name}!\n".inspect})"

		cons_call = CPP::FuncCall.new("new #{type_name}")
		cons_call.args.push(@loc.queue_name(@name).inspect)

		cons_call.args.push(@type.hash_with_name(@loc.queue_name(@name).inspect))
		@type.queues.collect do |queue|
			cons_call.args.push(@loc.queue_name(@name + "." + queue.name).inspect)
		end
		if_stmt.suite << CPP::Assign.new("#{@name}_ptr",cons_call)
		if_stmt.else_suite << CPP::FuncCall.build("printf","already made a #{@name}\n".inspect)


		destruct = CPP::Destructor.new(init_class)
		destruct.suite << if_stmt = CPP::If.new("0 == --#{counter}")
		if_stmt.suite << "printf(#{"deleting a #{@name}!! :) !!\n".inspect})"
		if_stmt.suite << "delete #{@name}_ptr"
		if_stmt.suite << CPP::Assign.new("#{@name}_ptr","NULL")

		init_class.add_member(:public,destruct)
		init_class.static = true
		init_class.dec = @name + "_initializer";

		str = <<COMMENT_END
Create a reference to the new object in the pointer.  Since we have already
created the initializer
COMMENT_END
		comments = str.split(/\n/).map{|str_sec| CPP::Comment.new(str_sec)}
		comments << "static UNUSED_VARIABLE #{type_name} &#{@name} = " +
		            "#{@name}_initializer.get()"
		namespace.add_post_swig("%immutable #{@name}_initializer")
		namespace.add_post_swig(CPP::SwigPragma.new("java", "modulecode", CPP::Suite.new(["public static final #{@name} = get#{@name.capitalize}_initializer().get()"])))
		ifdef_statement = CPP::IfnDef.new(CPP::Suite.new(comments))
		ifdef_statement.name = "SWIG"
		namespace.add(ifdef_statement)


		get = init_class.def_func(type_name,"get") #.add_dep(type)
		get.pre_func_types = "&"
		get.suite << CPP::Return.new("*#{@name}_ptr")
	end
end
