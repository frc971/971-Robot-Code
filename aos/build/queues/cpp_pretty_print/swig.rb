class CPP::SwigPragma
	attr_accessor :suite
	def initialize(language, pragmatype, suite = CPP::Suite.new())
		@suite = suite
	@language = language
	@pragmatype = pragmatype
	end
	def pp(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("%pragma(#{@language}) #{@pragmatype}=%{")
		state.endline()
		if(@suite.respond_to?(:pp_no_braces))
			@suite.pp_no_braces(state)
		else
			state.pp(@suite)
		end
		state.endline()
		state.needs_semi = false
		state.suppress_indent()
		state.print("%}")
		state.endline()
		state.endline()
	end
end
class CPP::SWIGBraces
	attr_accessor :suite
	def initialize(suite = CPP::Suite.new())
		@suite = suite
	end
	def pp(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("%{")
		state.endline()
		if(@suite.respond_to?(:pp_no_braces))
			@suite.pp_no_braces(state)
		else
			state.pp(@suite)
		end
		state.endline()
		state.needs_semi = false
		state.suppress_indent()
		state.print("%}")
		state.endline()
		state.endline()
	end
end
class CPP::SwigInclude
	attr_accessor :filename
	def initialize(filename)
		@filename = filename
	end
	def pp(state)
		state.needs_semi = false
		state.suppress_indent()
		state.print("%include #{@filename}")
		state.endline()
	end
end
