
class StructStmt 
	def initialize(name,suite)
		@name = name
		@suite = suite
	end
	def q_eval(locals)
		group = Target::StructDec.new(@name)
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
