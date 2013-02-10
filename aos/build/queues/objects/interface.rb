class MessageElementReq
	def initialize(type,name)
		@type = type
		@name = name
	end
	def self.parse(tokens)
		type = tokens.expect(:tWord).data
		name = tokens.expect(:tWord).data
		tokens.expect(:tSemi)
		return self.new(type,name)
	end 
end 
class QueueReq
	def initialize(name,type = nil)
		@name = name
		@type = type
	end
	def self.parse(tokens)
		type_or_name = tokens.expect(:tWord).data
		if(tokens.peak == :tSemi)
			tokens.expect(:tSemi)
			return self.new(type_or_name)
		else
			name = tokens.expect(:tWord).data
			tokens.expect(:tSemi)
			return self.new(name,type_or_name)
		end
	end
end
class InterfaceStmt < QStmt
	def initialize(name,elements)
		@name = name
		@elements = elements
	end
	def q_eval(locals)

	end
	def self.check_type(tokens,new_type,old_type)
		return new_type if(old_type == nil)
		if(new_type != old_type)
			tokens.qError(<<ERROR_MSG)
error: intermixing queue definitions (a queue_group feature)
	and type definitions (a message_group feature)
	this results in an undefined type value.
	Wot. Wot.
ERROR_MSG
		end
		return old_type
	end
	def self.parse(tokens)
		name = tokens.expect(:tWord).data
		values = []
		type = nil
		tokens.expect(:tOpenB)
		while(tokens.peak != :tCloseB)
			if(tokens.peak.data == "queue")
				tokens.expect(:tWord)
				values << QueueReq.parse(tokens)
				type = check_type(tokens,:queue_group,type)
			else
				values << MessageElementReq.parse(tokens)
				type = check_type(tokens,:message_group,type)
			end
		end 
		tokens.expect(:tCloseB)
		tokens.expect(:tSemi)
		self.new(name,values)
	end
end
