class QueueGroupTypeStmt < QStmt
	def initialize(name,suite)
		@name,@suite = name,suite
	end
	def q_eval(locals)
		group = Target::QueueGroupDec.new(@name)
		group.created_by = self
		locals.register(group)
		@suite.each do |stmt|
			stmt.q_eval(locals.bind(group))
		end
		return group
	end
end
class ImplementsStmt < QStmt
	def initialize(name)
		@name = name
	end
	def q_eval(locals)

	end
	def self.parse(tokens)
		name = QualifiedName.parse(tokens)
		tokens.expect(:tSemi)
		return self.new(name)
	end
end
class QueueGroupStmt < QStmt
	def initialize(type,name)
		@type,@name = type,name
	end
	def q_eval(locals)
		group = Target::QueueGroup.new(@type.lookup(locals),@name)
		group.created_by = self
		locals.register(group)
		return group
	end
	def self.parse(tokens)
		line = tokens.pos
		type_name = QualifiedName.parse(tokens)
		if(type_name.names.include?("queue_group"))
			tokens.qError(<<ERROR_MSG)
I was looking at the identifier you gave 
\tfor the queue group type between line #{line} and #{tokens.pos}
\tThere shouldn't be a queue_group type called queue_group
\tor including queue_group in it's path, it is a reserved keyword.
\tWot. Wot.
ERROR_MSG
		end
		if(tokens.peak == :tOpenB)
			if(type_name.is_simple?())
				type_name = type_name.to_simple
			else
				tokens.qError(<<ERROR_MSG)
You gave the name: "#{type_name.to_s}" but you're only allowed to 
\thave simple names like "#{type_name.names[-1]}" in queue_group definitions
\ttry something like:
\tqueue_group ControlLoop { }
\tWot. Wot.
ERROR_MSG
			end
			tokens.expect(:tOpenB)
			suite = []
			while(tokens.peak != :tCloseB)
				token = tokens.expect(:tWord) do |token|
					<<ERROR_MSG
I'm a little confused, I found a #{token.humanize} at #{token.pos}
\tbut what I really wanted was an identifier signifying a nested 
\tmessage declaration, or queue definition, or an impliments statement.
\tWot.Wot
ERROR_MSG
				end
				case token.data
				when "message"
					suite << MessageStmt.parse(tokens)
				when "queue"
					suite << QueueStmt.parse(tokens)
				when "implements"
					suite << ImplementsStmt.parse(tokens)
				else
					tokens.qError(<<ERROR_MSG)
expected a "queue","implements" or "message" statement rather
\tthan a #{token.data.inspect}.
\tWot. Wot.
ERROR_MSG
				end 
			end
			tokens.expect(:tCloseB)
			obj = QueueGroupTypeStmt.new(type_name,suite).set_line(line)
		else
			name = (tokens.expect(:tWord) do |token|
				<<ERROR_MSG
I found a #{token.humanize} at #{token.pos}
\tbut I was in the middle of parsing a queue_group statement, and
\twhat I really wanted was an identifier to store the queue group.
\tSomething like: queue_group control_loops.Drivetrain my_cool_group;
\tWot.Wot
ERROR_MSG
			end).data
			obj = QueueGroupStmt.new(type_name,name).set_line(line)
			if(tokens.peak == :tDot)
				tokens.qError(<<ERROR_MSG)
Hey! It looks like you're trying to use a complex identifier at: #{tokens.pos}
\tThats not going to work. Queue Group definitions have to be of the form:
\tqueue_group ComplexID SimpleID
\tWot. Wot.
ERROR_MSG
			end	
		end
		tokens.expect(:tSemi) do |token|
			token.pos
		end
		return obj
	end
end
