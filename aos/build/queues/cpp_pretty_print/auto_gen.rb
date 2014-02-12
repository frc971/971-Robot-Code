module CPP
end
class CPP::Comment
	def initialize(text)
		@text = text
	end
	def pp(state)
		state.needs_semi = false
		if(@text.include?("\n"))
			state.print("/* #{@text} */")
		else
			state.print("// #{@text}")
		end
	end
end
class CPP::TODO < CPP::Comment
	def initialize(owner,text)
		@text = "TODO(#{owner}): #{text}"
	end
end
class CPP::StaticVar
	class ForwardDec
		def initialize(func) ; @func = func ; end
		def pp(state) ; @func.pp_forward_dec(state) ; end
	end
	attr_accessor :args, :static
	def initialize(type_class, val_type, name, args = CPP::Args.new())
		@type_class = type_class
		@val_type = val_type
		@name = name
		@args = args
		@static = true
	end
	def forward_dec() ; ForwardDec.new(self) ; end
	def pp_forward_dec(state)
    if (@static)
      state.print("static ")
		end
		state.print("#{@val_type} #{@name}")
	end
	def pp(state)
		state.print("#{@val_type} #{@type_class.chop_method_prefix}#{@name}(")
		state.pp(@args)
		state.print(")")
	end
	alias_method :pp_header_file, :pp_forward_dec
	alias_method :pp_cc_file, :pp
end
class CPP::MemberFunc
	class ForwardDec
		def initialize(func) ; @func = func ; end
		def pp(state) ; @func.pp_forward_dec(state) ; end
	end
	attr_accessor :args,:suite,:return_type,:name,:pre_func_types,:const,:static,:virtual
	def initialize(type_class,return_type,name)
		@type_class = type_class
		@return_type = return_type
		@name = name
		@const = false
		@static = false
		@virtual = false
		@args = CPP::Args.new()
		@suite = CPP::Suite.new()
	end
	attr_accessor :inline
	def forward_dec() ; ForwardDec.new(self) ; end
	def pp_forward_dec(state)
		return self.pp_inline(state) if(@inline)
        if (@static)
          state.print("static ")
				elsif (@virtual)
          state.print("virtual ")
        end
		state.print("#{@return_type} #{@pre_func_types}#{@name}(")
		state.pp(@args)
		state.print(")")
        if (@const)
          state.print(" const")
        end
	end
	def pp_inline(state)
        if (@static)
          state.print("static ")
				elsif (@virtual)
          state.print("virtual ")
        end
		state.print("#{@return_type} #{@pre_func_types}#{@name}(")
		state.pp(@args)
		state.print(") ")
        if (@const)
          state.print(" const")
        end
		@suite.pp_one_line(state)
	end
	def pp(state)
		return if(@inline)
		state.print("#{@return_type} #{@pre_func_types}#{@type_class.chop_method_prefix}#{@name}(")
		state.pp(@args)
		state.print(") ")
		if (@const)
			state.print("const ")
		end
		state.pp(@suite)
		state.v_pad(2)
	end
	alias_method :pp_header_file, :pp_forward_dec
	alias_method :pp_cc_file, :pp
	
end
class CPP::Constructor
	class ForwardDec
		def initialize(func) ; @func = func ; end
		def pp(state) ; @func.pp_forward_dec(state) ; end
	end
	attr_accessor :args,:suite,:return_type,:name
	def initialize(type_class)
		@type_class = type_class
		@args = CPP::Args.new()
		@suite = CPP::Suite.new()
		@var_cons = CPP::Args.new()
	end
	def forward_dec() ; ForwardDec.new(self) ; end
	def add_cons(*args)
		@var_cons.push(CPP::FuncCall.build(*args))
	end
	def pp_forward_dec(state)
		state.print("#{@type_class.name}(")
		state.pp(@args)
		state.print(")")
	end
	def pp_inline(state)
		pp(state,false)
	end
	def pp(state,prefix = true)
		state.needs_semi = false
		state.print(@type_class.chop_method_prefix) if(prefix)
		state.print("#{@type_class.name}(")
		state.pp(@args)
		if(@var_cons.length >= 1)
			state.print(")")
			state.endline()
			state.indent += 4
			state.print(": ")
			state.pp(@var_cons)
			state.indent -= 4
			state.print(" ")
		else
			state.print(") ")
		end
		state.pp(@suite)
		state.v_pad(2)
	end
	alias_method :pp_header_file, :pp_forward_dec
	alias_method :pp_cc_file, :pp
end
class CPP::Destructor
	class ForwardDec
		def initialize(func) ; @func = func ; end
		def pp(state) ; @func.pp_forward_dec(state) ; end
	end
	attr_accessor :args,:suite,:return_type,:name
	def initialize(type_class)
		@type_class = type_class
		@args = CPP::Args.new()
		@suite = CPP::Suite.new()
	end
	def forward_dec() ; ForwardDec.new(self) ; end
	def pp_forward_dec(state)
		state.print("~#{@type_class.name}(")
		state.pp(@args)
		state.print(")")
	end
	def pp(state)
		state.print("#{@type_class.chop_method_prefix}~#{@type_class.name}(")
		state.pp(@args)
		state.print(") ")
		state.pp(@suite)
		state.v_pad(2)
	end
	alias_method :pp_header_file, :pp_forward_dec
	alias_method :pp_cc_file, :pp
end
class CPP::Include
	attr_accessor :filename
	def initialize(filename)
		@filename = filename
	end
	def pp(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("#include #{@filename}")
		state.endline()
	end
end
class CPP::IncludeGuard
	attr_accessor :name,:suite
	def initialize(suite = CPP::Suite.new())
		@suite = suite
	end
	def self.rand_name(len = 40)
		str = ""
		len.times { str += ((rand(26) + ?A).chr)}
		return str
	end
	def pp(state)
		@name ||= IncludeGuard.rand_name
		state.needs_semi = false
		state.suppress_indent()
		state.print("#ifndef #{@name}")
		state.endline()
		state.suppress_indent()
		state.print("#define #{@name}")
		state.endline()
		if(@suite.respond_to?(:pp_no_braces))
			@suite.pp_no_braces(state)
		else
			state.pp(@suite)
		end
		state.endline()
		state.needs_semi = false
		state.suppress_indent()
		state.print("#endif  // #{@name}")
		state.endline()
	end
end
class CPP::IfnDef
	attr_accessor :name,:suite
	def initialize(suite = CPP::Suite.new())
		@suite = suite
	end
	def pp(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("#ifndef #{@name}")
		state.endline()
		if(@suite.respond_to?(:pp_no_braces))
			@suite.pp_no_braces(state)
		else
			state.pp(@suite)
		end
		state.endline()
		state.needs_semi = false
		state.suppress_indent()
		state.print("#endif  // #{@name}")
		state.endline()
	end
end
class CPP::PreprocessorIf
	attr_accessor :name,:suite
	def initialize(ifsuite, elsesuite)
		@ifsuite = ifsuite
		@elsesuite = elsesuite
	end
	def write_if(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("#if #{@name}")
		state.endline()
	end
	def write_else(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("#else  // #{@name}")
		state.endline()
	end
	def write_endif(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("#endif  // #{@name}")
		state.endline()
	end
	def pp_inline(state)
		self.write_if(state)
		@ifsuite.pp_inline(state)
		if(@elsesuite != nil)
			self.write_else(state)
			@elsesuite.pp_inline(state)
		end
		self.write_endif(state)
	end
	def pp(state)
		self.write_if(state)
		if(@ifsuite.respond_to?(:pp_no_braces))
			@ifsuite.pp_no_braces(state)
		else
			state.pp(@ifsuite)
		end
		if(@elsesuite != nil)
			self.write_else(state)
			if(@elsesuite.respond_to?(:pp_no_braces))
				@elsesuite.pp_no_braces(state)
			else
				state.pp(@elsesuite)
			end
		end
		self.write_endif(state)
	end
end
class CPP::Using
	def initialize(using)
		@using = using
	end
	def pp(state)
		state.print("using #{@using}")
	end
end
