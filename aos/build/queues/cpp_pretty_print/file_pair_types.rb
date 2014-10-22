module CPP
end
class CPP::TypePair
	attr_accessor :name,:static,:dec
	def initialize(namespace,name)
		@space,@name = namespace,name
		@members = []
		@protections = {}
		@dont_sort = {}
	end
	ProtectionTable = { :public => 0,:protected => 1,:private => 2 }
	def comp(m1,m2)
		if(!(@dont_sort[m1] || @dont_sort[m2]))
			if(@protections[m1] && @protections[m2])
				comp = ProtectionTable[@protections[m1]] <=> ProtectionTable[@protections[m2]]
				return comp if(comp != 0)
			end
			comp = TypeTable[m1.class] <=> TypeTable[m2.class]
			return comp if(comp != 0)
		end
		return @stable[m1] <=> @stable[m2]
	end
	def set_parent(parent)
		@parent = parent
	end
	def add_class(name)
		return add_member(:public,CPP::ClassPair.new(self,name))
	end
	def add_struct(name)
		return add_member(:public,CPP::StructPair.new(self,name))
	end
	def add_cc_comment(*strs)
		strs.each do |str|
			@members << comment = CPP::CCMemberWrapper.new(CPP::Comment.new(str))
			@protections[comment] = @protection
		end
		#@dont_sort[comment] = true
	end
	def method_prefix()
		@space.method_prefix + "#@name::"
	end
	def chop_method_prefix()
		@space.chop_method_prefix + "#@name::"
	end
	def pp(state)
		@stable = {}
		@members.each_with_index { |mem,i| @stable[mem] = i }
		members = @members.sort { |m1,m2| comp(m1,m2) }

		state.needs_semi = false
		state.v_pad(2)
		if(@static)
			state.print("static ")
		end
		if(@parent)
			state.print("#{self.class.type_name} #{@name} : #{@parent} {\n")
		else
			state.print("#{self.class.type_name} #{@name} {\n")
		end 
		state.indent += 2
		protection = nil 
		members.each do |member|
			if(protection != @protections[member])
				state.indent -= 1
				state.needs_semi = false
				state.v_pad(2) if(protection)
				protection = @protections[member]
				state.print("#{protection}:\n")
				state.indent += 1
				state.endline()
			end
			state.endline()
			state.needs_semi = true
			if(member.respond_to?(:pp_header_file))
				member.pp_header_file(state)
			else
				state.pp(member)
			end
			state.endline()
		end 
		state.indent -= 2
		state.needs_semi = true
		if(@dec)
		state.print("\n} #{@dec}")
		else
		state.print("\n}")
		end
		state.v_pad(2)
	end
	def pp_header_file(state)
		pp(state)
	end
	def pp_cc_file(state)
		@members.each do |member|
			state.needs_semi = true
			member.pp_cc_file(state) if(member.respond_to?(:pp_cc_file))
			state.endline()
		end
	end
	def def_func(*args)
		func = CPP::MemberFunc.new(self,*args)
		@protections[func] = @protection
		@members << func
		return func
	end
end
class CPP::CCMemberWrapper
	def initialize(member)
		@member = member
	end
	def pp_header_file(state) ; end
	def pp_cc_file(state)
		state.pp(@member)
	end
end
class CPP::ClassPair < CPP::TypePair
	attr_accessor :name
	def initialize(namespace,name)
		super(namespace,name)
		@protection = :public #default to public
	end
	def add_member(protection,member)
		@protection = protection
		@members << member
		@protections[member] = protection
		return member
	end
	def public() ; @protection = :public ; end
	def protected() ; @protection = :protected ; end
	def private() ; @protection = :private ; end
	def self.type_name() ; "class" ; end
end
class CPP::StructPair < CPP::TypePair
	def self.type_name() ; "struct" ; end
	def add_member(member)
		@members << member
		return member
	end
end
CPP::TypePair::TypeTable = { CPP::StructPair => 0, CPP::ClassPair => 1,CPP::Constructor => 2 }
CPP::TypePair::TypeTable.default = 3
class CPP::TemplateClass < CPP::ClassPair
	def initialize(namespace,name)
		super(namespace,name)
		@t_args = CPP::Args.new()
	end
	def spec_args()
		return @spec_args ||= CPP::Args.new()
	end
	def pp(state)
		@stable = {}
		@members.each_with_index { |mem,i| @stable[mem] = i }
		members = @members.sort { |m1,m2| comp(m1,m2) }
		state.needs_semi = false
		state.v_pad(2)
		if(@t_args.length > 0)
			state.pp("template < ")
			state.pp(@t_args)
		else
			state.pp("template < ")
		end
		state.pp(">\n")
		state.pp("class #@name")
		if(@spec_args)
			state.pp("< ")
			state.pp(@spec_args)
			state.pp("> {")
		else
			state.pp(" {")
		end
		state.endline()
		state.indent += 2
		protection = nil 
		members.each do |member|
			if(protection != @protections[member])
				state.indent -= 1
				state.needs_semi = false
				state.v_pad(2) if(protection)
				protection = @protections[member]
				state.print("#{protection}:\n")
				state.indent += 1
				state.endline()
			end
			state.endline()
			state.needs_semi = true
			if(member.respond_to?(:pp_inline))
				member.pp_inline(state)
			else
				state.pp(member)
			end
			state.endline()
		end 
		state.indent -= 2
		state.needs_semi = true
		if(@dec)
			state.print("\n} #{@dec}")
		else
			state.print("\n}")
		end
		state.v_pad(2)
	end
	def pp_cc_file(state); end
end

