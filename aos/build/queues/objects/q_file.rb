class QStmt
	def set_line(val)
		@file_line = val
		return self
	end
	def q_stack_name()
		@file_line
	end
	def self.method_added(name)
		@wrapped ||= {}
		return if(name != :q_eval && name != :q_eval_extern)
		return if(@wrapped[name])
		@wrapped[name] = true
		method = self.instance_method(name)
		define_method(name) do |*args,&blk|
			begin 
				method.bind(self).call(*args,&blk)
			rescue QError => e
				e.qstacktrace << self
				raise e
			end 
		end 
	end 

end
class ImportStmt < QStmt
	def initialize(filename)
		@filename = filename
	end
	def q_eval(locals)
		filename = locals.globals.find_file(@filename)
		#puts "importing #{filename.inspect}"
		q_file = QFile.parse(filename)
		q_output = q_file.q_eval_extern(locals.globals)
		q_output.extern = true
		return Target::QInclude.new(@filename + ".h")
	end
	def self.parse(tokens)
		line = tokens.pos
		to_import = (tokens.expect(:tString) do |token|
			<<ERROR_MSG
I found a #{token.humanize} at #{token.pos}.
\tI was really looking for a "filename" for this import statement.
\tSomething like: import "super_cool_file";
\tWot.Wot
ERROR_MSG
		end).data
		tokens.expect(:tSemi) do |token|
			<<ERROR_MSG
I found a #{token.humanize} at #{token.pos}.
\tI was really looking for a ";" to finish off this import statement
\tat line #{line};
\tSomething like: import #{to_import.inspect};
\tWot.Wot                #{" "*to_import.inspect.length}^
ERROR_MSG
		end
		return self.new(to_import).set_line(line)
	end
end
class PackageStmt < QStmt
	def initialize(name)
		@name = name
	end
	def q_eval(locals)
		locals.package = @name
		return nil
	end
	def self.parse(tokens)
		line = tokens.pos
		qualified_name = QualifiedName.parse(tokens)
		tokens.expect(:tSemi)
		return self.new(qualified_name).set_line(line)
	end
end
class QFile
	attr_accessor :namespace
	def initialize(filename,suite)
		@filename,@suite = filename,suite
	end
	def q_eval(globals = Globals.new())
		local_pos = LocalSituation.new(globals)
		q_file = Target::QFile.new()
		@suite.each do |name|
			val = name.q_eval(local_pos)
			if(val)
				if(val.respond_to?(:create))
					q_file.add_type(val)
				end
			end
		end
		@namespace = local_pos.local
		return q_file
	end
	def q_eval_extern(globals)
		local_pos = LocalSituation.new(globals)
		q_file = Target::QFile.new()
		@suite.each do |name|
			if(name.respond_to?(:q_eval_extern))
				val = name.q_eval_extern(local_pos)
			else
				val = name.q_eval(local_pos)
			end
			if(val)
				if(val.respond_to?(:create))
					q_file.add_type(val)
				end
			end
		end
		return q_file
	end
	def self.parse(filename)
		tokens = Tokenizer.new(filename)
		suite = []
		while(tokens.peak != :tEnd)
			token = tokens.expect(:tWord) do |token| #symbol
			<<ERROR_MSG
I found a #{token.humanize} at #{token.pos}.
\tI was really looking for a "package", "import", "queue_group",
\t"message", or "queue" statement to get things moving.
\tSomething like: import "super_cool_file";
\tWot.Wot
ERROR_MSG
			end
			case token.data
			when "package"
				suite << PackageStmt.parse(tokens)
			when "import"
				suite << ImportStmt.parse(tokens)
			when "queue_group"
				suite << QueueGroupStmt.parse(tokens)
			when "message"
				suite << MessageStmt.parse(tokens)
			when "queue"
				suite << QueueStmt.parse(tokens)
			when "interface"
				suite << InterfaceStmt.parse(tokens)
			when "struct"
				suite << StructStmt.parse(tokens)
			else
				tokens.qError(<<ERROR_MSG)
expected a "package","import","queue","struct","queue_group", or "message" statement rather
	than a #{token.data.inspect}, (whatever that is?)
	oh! no! a feature request!?
	Wot. Wot.
ERROR_MSG
			end
		end
		return self.new(filename,suite)
	end
end
