class QError < Exception
	def initialize(msg)
		super()
		@msg = msg
		@qstacktrace = []
	end
	def self.set_name(name)
		@pretty_name = name
	end
	def self.pretty_name()
		@pretty_name
	end
	def to_s
		msg = "Error:(#{self.class.pretty_name})\n\t"
		msg += @msg
		msg += "\n" if(msg[-1] != "\n")
		@qstacktrace.each do |part|
			part = part.q_stack_name if(part.respond_to?(:q_stack_name))
			msg += "\tfrom: #{part}\n"
		end
		return msg
	end
	set_name("Base Level Exception.")
	attr_accessor :qstacktrace
end
class QSyntaxError < QError
	def initialize(msg)
		super(msg)
	end
	set_name("Syntax Error")
end
class QNamespaceCollision < QError
	def initialize(msg)
		super(msg)
	end
	set_name("Namespace Collision")
end
class QImportNotFoundError < QError
	def initialize(msg)
		super(msg)
	end
	set_name("Couldn't Find Target of Import Statement")
end
