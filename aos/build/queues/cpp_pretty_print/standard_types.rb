module CPP
	class Class

	end
	class Funct
		attr_accessor :retval,:name,:args,:suite
		def initialize(retval = nil,name = nil,args = Args.new(),suite = Suite.new())
			@retval,@name,@args,@suite = retval,name,args,suite
		end
		def pp(state)
			state.pp(@retval)
			state.print(" ")
			state.pp(@name)
			state.print("(")
			state.pp(@args)
			state.print(")")
			state.pp(@suite)
		end
	end
	class If
		attr_accessor :cond,:suite,:else_suite
		def initialize(cond = nil,suite = Suite.new(),else_suite = nil)
			@cond,@suite,@else_suite = cond,suite,else_suite
		end
		def pp(state)
			state.print("if (")
			state.pp(@cond)
			state.print(") ")
			state.needs_semi = false
			state.pp(@suite)
			if(@else_suite)
				state.print(" else ")
				state.pp(@else_suite)
			end
		end
		def else_suite()
			return(@else_suite ||= Suite.new())
		end
	end
	class For
		attr_accessor :init,:cond,:update,:suite
		def initialize(init = nil,cond = nil,update = nil,suite = Suite.new())
			@init,@cond,@update,@suite = init,cond,update,suite
		end
		def pp(state)
			state.print("for (")
			Args.new([@init,@cond,@update]).pp(state,";")
			state.print(") ")
			state.needs_semi = false
			state.pp(@suite)
		end
	end
	class Args < Array
		attr_accessor :dont_wrap
		def pp(state,sep = ",")
			pos = start = state.col
			self.each_with_index do |arg,i|
				#puts "#{state.col - start} , #{start}"
				state.print(sep) if(i != 0)
				if(pos > 80)
					state.wrap(state.indent + 4)
					pos = start = state.col
				elsif(state.col * 2 - pos > 80 && !@dont_wrap)
					state.wrap(start)
					start = pos
				else
					state.print(" ") if(i != 0)
					pos = state.col
				end
				state.pp(arg)
			end
		end
	end
	class Suite < Array
		def pp(state)
			state.print("{")
			state.>>
			state.needs_semi = false
			self.pp_no_braces(state)
			state.<<
			state.print("}")
		end
		def pp_no_braces(state)
			self.each do |arg,i|
				state.endline()
				state.needs_semi = true
				state.pp(arg)
				state.endline()
			end
		end
		def pp_one_line(state)
			if(self.length == 1)
				state.needs_semi = true
				state.print("{ ")
				state.pp(self[0])
				state.print(";") if(state.needs_semi)
				state.needs_semi = false
				state.print(" }")
			else
				self.pp(state)
			end
		end
	end
	class FuncCall
		attr_accessor :name,:args
		def initialize(name = nil,args = Args.new())
			@name,@args = name,args
		end
		def self.build(name,*args)
			self.new(name,Args.new(args))
		end
		def pp(state)
			state.pp(@name)
			state.print("(")
			state.pp(@args)
			state.print(")")
		end
	end
	class BIT_OR
		attr_accessor :val1,:val2
		def initialize(val1,val2)
			@val1,@val2 = val1,val2
		end
		def pp(state)
			state.pp(@val1)
			state.print(" | ")
			state.pp(@val2)
		end
	end
	class LT
		attr_accessor :val1,:val2
		def initialize(val1,val2)
			@val1,@val2 = val1,val2
		end
		def pp(state)
			state.pp(@val1)
			state.print(" < ")
			state.pp(@val2)
		end
	end
	class Div
		attr_accessor :val1,:val2
		def initialize(val1,val2)
			@val1,@val2 = val1,val2
		end
		def pp(state)
			state.pp(@val1)
			state.print(" / ")
			state.pp(@val2)
		end
	end
	class Add
		attr_accessor :val1,:val2
		def initialize(val1,val2)
			@val1,@val2 = val1,val2
		end
		def pp(state)
			state.pp(@val1)
			state.print(" + ")
			state.pp(@val2)
		end
	end
	class Member
		attr_accessor :obj,:name
		def initialize(obj,name)
			@obj,@name = obj,name
		end
		def pp(state)
			state.pp(@obj)
			state.print(".")
			state.pp(@name)
		end
	end
	class PointerMember
		attr_accessor :obj,:name
		def initialize(obj,name)
			@obj,@name = obj,name
		end
		def pp(state)
			state.pp(@obj)
			state.print("->")
			state.pp(@name)
		end
	end
	class Minus
		attr_accessor :val
		def initialize(val)
			@val = val
		end
		def pp(state)
			state.print("-")
			state.pp(@val)
		end
	end
	class Not
		attr_accessor :val
		def initialize(val)
			@val = val
		end
		def pp(state)
			state.print("!")
			state.pp(@val)
		end
	end
	class Address
		attr_accessor :val
		def initialize(val)
			@val = val
		end
		def pp(state)
			state.print("&")
			state.pp(@val)
		end
	end
	class Paran
		attr_accessor :val
		def initialize(val)
			@val = val
		end
		def pp(state)
			state.print("(")
			state.pp(@val)
			state.print(")")
		end
	end
	class Assign
		attr_accessor :lval,:rval
		def initialize(lval = nil,rval = Args.new())
			@lval,@rval = lval,rval
		end
		def pp(state)
			state.pp(@lval)
			state.print(" = ")
			state.pp(@rval)
		end
	end
	class PointerDeref
		attr_accessor :val
		def initialize(val)
			@val = val
		end
		def pp(state)
			state.print("*")
			state.pp(@val)
		end
	end
	class Cast
		attr_accessor :to,:val
		def initialize(to,val)
			@to,@val = to,val
		end
		def pp(state)
			state.print("(")
			state.pp(@to)
			state.print(")")
			state.pp(@val)
		end
	end
	class VarDec
		attr_accessor :type,:name
		def initialize(type = nil,name = nil)
			@type,@name = type,name
		end
		def pp(state)
			state.pp(@type)
			state.print(" ")
			state.pp(@name)
		end
	end
	class Return
		attr_accessor :val
		def initialize(val = nil)
			@val = val
		end
		def pp(state)
			state.print("return ")
			state.pp(@val)
		end
	end
	class TraversalState
		attr_accessor :needs_semi,:indent,:col
		def initialize(file)
			@file = file
			@indent = 0
			@not_indented = true
			@just_endlined = true
			@hold_points = []
			@col = 0
		end
		def set_hold_point()
			@hold_points.push(@col)
		end
		def wrap(col)
			@file.print("\n")
			@file.print(" " * col)
			@col = col
		end
		def suppress_indent()
			@not_indented = false
		end
		def <<()
			@indent -= 2
		end
		def >>()
			@indent += 2
		end
		def pp(ast)
			return ast.pp(self) if(ast.respond_to?(:pp))
			self.print(ast)
		end
		def print(chars)
			return print_no_split(chars) if(!chars.respond_to?(:split))
			chars.split(/\n/,-1).each_with_index do |line,i|
				endline() if(i != 0)
				print_no_split(line) if(line && line.length > 0)
			end
		end
		def print_no_split(chars)
			if(@not_indented)
				@file.print(" "*@indent)
				@col += @indent
				@not_indented = false
			end
			if(chars)
				chars = chars.to_s
				if(chars.length > 0)
					@col += chars.length
					@file.print(chars)
					@just_endlined = false
					@v_pad = 0
				end
			end
		end
		def endline()
			return if(@just_endlined)
			@file.print(";") if(@needs_semi)
			@needs_semi = false
			@file.print("\n")
			@not_indented = true
			@just_endlined = true
			@hold_points.clear()
			@v_pad += 1
			@col = 0
		end
		def v_pad(n)
			while(@v_pad < n)
				force_endline()
			end
		end
		def force_endline()
			@just_endlined = false
			endline()
		end
	end
	def self.pretty_print(ast,file)
		state = TraversalState.new(file)
		if(ast.respond_to?(:pp_no_braces))
			ast.pp_no_braces(state) 
		else
			state.pp(ast)
		end
	end
end
