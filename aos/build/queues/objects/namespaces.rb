class LocalSituation
	attr_accessor :globals,:local
	def initialize(globals)
		@globals = globals
		@local = nil
	end
	def package=(qualified)
		if(@local)
			raise QSyntaxError.new(<<ERROR_MSG)
You are redefining the package path.
 Stuff might break if you do that.
 Other options include: using another header file, and just using the same namespace.
 If you are confident that you need this you can remove this check at
 	#{__FILE__}:#{__LINE__}.
 Or file a feature request.
 But that would be weird...
 Wot. Wot.
ERROR_MSG
		end
		@local = @globals.space
		qualified.names.each do |name|
			@local = @local.get_make(name)
		end
	end
	def register(value)
		if(!@local)
			raise QError.new(<<ERROR_MSG)
There is no package path defined, This is a big problem because
 we are kindof expecting you to have a package path...
 use a :
    package my_super.cool.project;
 statement to remedy this situation. (or file a feature request)
 Wot. Wot.
ERROR_MSG
		end
		@local[value.name] = value
		value.parent = @local if(value.respond_to?(:parent=))
		value.loc = @local
	end
	def bind(bind_to)
		return BoundSituation.new(self,bind_to)
	end
end
class BoundSituation < LocalSituation
	def initialize(locals,bind_to)
		@globals = locals.globals
		@local = bind_to
	end
end
class NameSpace
	attr_accessor :parent,:name
	def initialize(name = nil,parent = nil)
		@name = name
		@parent = parent
		@spaces = {}
	end
	def []=(key,val)
		if(old_val = @spaces[key])
			old_val = old_val.created_by if(old_val.respond_to?(:created_by) && old_val.created_by)
			if(old_val.respond_to?(:q_stack_name))
				old_val = old_val.q_stack_name
			else
				old_val = "eh, it is a #{old_val.class} thats all I know..."
			end
			raise QNamespaceCollision.new(<<ERROR_MSG)
Woah! The name #{queue_name(key).inspect} is already taken by some chap at #{old_val}.
\tFind somewhere else to peddle your wares.
\tWot. Wot.
ERROR_MSG
		end
		@spaces[key] = val
	end
	def to_cpp_id(name)
		txt = @name + "::" + name
		return @parent.to_cpp_id(txt) if(@parent && parent.name)
		return "::" + txt
	end
	def queue_name(queue)
		get_name() + "." + queue
	end
	def [](key)
		#puts "getting #{get_name}.#{key}"
		@spaces[key]
	end
	def get_make(name)
		@spaces[name] ||= self.class.new(name,self)
	end
	def get_name()
		if(@parent)
			return "#{@parent.get_name}.#{@name}"
		else
			return "#{@name}"
		end
	end
	def create(cpp_tree)
		if(@parent && @parent.name)
			parent = cpp_tree.get(@parent)
		else
			parent = cpp_tree
		end
		return cpp_tree.add_namespace(@name,parent)
	end
	def to_s()
		"<NameSpace: #{get_name()}>"
	end
	def inspect()
		"<NameSpace: #{get_name()}>"
	end
	def root()
		return self if(@parent == nil)
		return @parent.root
	end
end
class Globals
	attr_accessor :space
	def initialize()
		@space = NameSpace.new()
		@space.get_make("aos")
		@include_paths = []
	end
	def paths()
		@include_paths
	end
	def add_path(path)
		@include_paths << path
	end
	def find_file(filename)
		@include_paths.each do |path_name|
			new_path = File.expand_path(path_name) + "/" + filename
			if(File.exists?(new_path))
				return new_path 
			end
		end
		raise QImportNotFoundError.new(<<ERROR_MSG)
Problem Loading:#{filename.inspect} I looked in:
\t#{(@include_paths.collect {|name| name.inspect}).join("\n\t")}
\tbut alas, it was nowhere to be found.
\tIt is popular to include the top of the repository as the include path start, 
\tand then reference off of that. 
\tI would suggest doing that and then trying to build again.
\tWot. Wot.
ERROR_MSG
	end
end
class QualifiedName
	attr_accessor :names,:off_root
	def initialize(names,off_root = false)
		@names = names
		@off_root = off_root
	end
	def test_lookup(namespace)
		@names.each do |name|
			return nil if(!namespace.respond_to?(:[]))
			namespace = namespace[name]
			return nil if(!namespace)
		end
		return namespace
	end
	def is_simple?()
		return !@off_root && @names.length == 1
	end
	def to_simple()
		return @names[-1]
	end
	def to_s()
		if(@off_root)
			return ".#{@names.join(".")}"
		else
			return @names.join(".")
		end
	end
	def lookup(locals)
		if(@off_root)
			local = locals.globals.space
		else
			local = locals.local
		end
		target = nil
		while(!target && local)
			target = test_lookup(local)
			local = local.parent
		end
		return target if(target)
		if(@off_root)
			raise QError.new(<<ERROR_MSG)
I was looking for .#{@names.join(".")}, but alas, it was not under
\tthe root namespace.
\tI'm really sorry old chap.
\tWot. Wot.
ERROR_MSG
		else
			raise QError.new(<<ERROR_MSG)
I was looking for #{@names.join(".")}, but alas, I could not find
\tit in #{locals.local.get_name} or any parent namespaces.
\tI'm really sorry old chap.
\tWot. Wot.
ERROR_MSG
		end
	end
	def self.parse(tokens)
		names = []
		off_root = (tokens.peak == :tDot)
		tokens.next if(off_root)
		names << tokens.expect(:tWord).data
		while(tokens.peak == :tDot)
			tokens.next
			names << tokens.expect(:tWord).data
		end
		return self.new(names,off_root)
	end
end
