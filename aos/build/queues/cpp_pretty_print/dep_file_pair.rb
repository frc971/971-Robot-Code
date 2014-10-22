class GroupElement
	def add_group_dep(group_dep)
		@group_dep = group_dep
	end
	def adjust_group(state,old_group)
		if(@group_dep != old_group)
			@group_dep.adjust_from(state,old_group)
		end
		return @group_dep
	end
end
class DepMask < GroupElement
	def initialize(elem)
		@elem = elem
		@deps = []
	end
	def add_dep(dep)
		@deps << dep
		self
	end
	def method_missing(method,*args,&blk)
		@elem.send(method,*args,&blk)
	end
	alias_method :old_respond_to, :respond_to?
	def respond_to?(method)
		old_respond_to(method) || @elem.respond_to?(method)
	end
end

class MemberElementHeader < GroupElement
	def initialize(elem)
		@elem = elem
	end
	def pp(state)
		if(@elem.respond_to?(:pp_header_file))
			@elem.pp_header_file(state)
		else
			state.pp(@elem)
		end
	end
	def self.check(plan,elem)
		plan.push(self.new(elem))
	end
end
class MemberElementInline < GroupElement
	def initialize(elem)
		@elem = elem
	end
	def pp(state)
		if(@elem.respond_to?(:pp_inline))
			@elem.pp_inline(state)
		else
			state.pp(@elem)
		end
	end
end
class MemberElementCC < GroupElement
	attr_accessor :pp_override
	def initialize(elem)
		@elem = elem
	end
	def pp(state)
		return state.pp(@elem) if(@pp_override)
		@elem.pp_cc_file(state)
	end
	def self.check(plan,elem)
		plan.push(self.new(elem)) if(elem.respond_to?(:pp_cc_file))
	end
end
class ImplicitName
	attr_accessor :parent
	def initialize(name,parent)
		@name = name
		@parent = parent
	end
	def close(state)
		state.endline()
		state.needs_semi = false
		state.print("}  // namespace #{@name}\n")
	end
	def name()
		if(@parent)
			return @parent.name + "." + @name
		else
			return "." + @name
		end
	end
	def open(state)
		state.endline()
		state.needs_semi = false
		state.print("namespace #{@name} {\n")
	end
	def adjust_from(state,old_group)
		close_grps = []
		grp = old_group
		while(grp)
			close_grps << grp
			grp = grp.parent
		end
		open_grps = []
		grp = self
		while(grp)
			open_grps << grp
			grp = grp.parent
		end
		while(open_grps[-1] == close_grps[-1] && close_grps[-1])
			close_grps.pop()
			open_grps.pop()
		end
		close_grps.each do |grp|
			grp.close(state)
		end
		open_grps.reverse.each do |grp|
			grp.open(state)
		end
	end
	def adjust_to(state,new_group)
		grp = self
		while(grp)
			grp.close(state)
			grp = grp.parent
		end
	end
end
class GroupPlan < GroupElement
	attr_accessor :implicit
	def initialize(group_elem,members = [])
		@group_elem = group_elem
		@members = CPP::Suite.new(members)
	end
	def push(mem)
		mem.add_group_dep(@implicit) if(@implicit)
		@members << mem
	end
	def pp(state)
		if(@group_elem)
			@group_elem.open(state)
		end
		group = nil
		@members.each do |member|
			group = member.adjust_group(state,group)
			#puts "at:#{group.name}" if(group.respond_to?(:name))
			state.endline()
			state.needs_semi = true
			state.pp(member)
			state.endline()
		end
		group.adjust_to(state,nil) if(group)
		if(@group_elem)
			@group_elem.close(state)
		end
	end
end
class CC_Mask
	def initialize(elem)
		@elem = elem
	end
	def plan_cc(plan)
		elem = MemberElementCC.new(@elem)
		elem.pp_override = true
		plan.push(elem)
	end
	def plan_header(plan);
	end
end
module Types
	class TypeDef
		def initialize(type,name)
			@type = type
			@name = name
		end
		def pp(state)
			state.pp("typedef #{@type.name} #{@name}")
		end
	end
	class Type
		attr_accessor :name,:static,:dec,:space
		def initialize(namespace,name)
			@space = namespace
			@name = name
			@members = []
			@protections = {}
			@deps = []
		end
    def parent_class
      @parent.split(' ')[1] if @parent
    end
		class ProtectionGroup
			def initialize(name)
				@name = name
			end
			def adjust_from(state,other)
				state.indent -= 1
				state.pp("#@name:")
				state.endline
				state.indent += 1
			end
			def adjust_to(state,other)
				other.adjust_from(state,self) if other
			end
		end
		ProtectionGroups = { :public => ProtectionGroup.new(:public),
			:protected => ProtectionGroup.new(:protected),
		 	:private => ProtectionGroup.new(:private)}
		def set_parent(parent)
			@parent = parent
		end
		def chop_method_prefix()
			@space.chop_method_prefix + "#@name::"
		end
		def def_func(*args)
			func = CPP::MemberFunc.new(self,*args)
			@protections[func] = @protection
			@members << func
			return func
		end
		def add_cc_comment(*args)
			args.each do |arg|
				@members.push(CC_Mask.new(CPP::Comment.new(arg)))
			end
		end
		def plan_cc(plan)
			@members.each do |member|
				if(member.respond_to?(:plan_cc))
					member.plan_cc(plan)
				elsif(member.respond_to?(:pp_cc_file))
					plan.push(MemberElementCC.new(member))
				end
			end
		end
		def plan_header(plan)
			group_plan = GroupPlan.new(self)
			plan.push(group_plan)
			@members.each do |member|
				group_plan.implicit = ProtectionGroups[@protections[member]]
				if(member.respond_to?(:plan_header))
					member.plan_header(group_plan)
				else
					group_plan.push(MemberElementHeader.new(member))
				end
				#member.plan_cc(plan)
			end
		end
		def open(state)
			state.needs_semi = false
			state.v_pad(2)
			if(@static)
				state.print("static ")
			end
			self.template_header(state) if(self.respond_to?(:template_header))
			state.print("#{self.class.type_name} #{@name}")
			self.template_spec_args(state) if(self.respond_to?(:template_spec_args))
			if(@parent)
				state.print(" : #{@parent} {\n")
			else
				state.pp(" {\n")
			end
			state.indent += 2
		end
		def close(state)
			state.indent -= 2
			state.needs_semi = true
			if(@dec)
				state.print("\n} #{@dec}")
			else
				state.print("\n}")
			end
			state.v_pad(2)
		end
		def add_dep(other)
			@deps << other
			self
		end
	end
	class Class < Type
		def add_member(protection,member)
			@protection = protection
			@members.push(member)
			@protections[member] = protection
			return member
		end
		def add_struct(*args)
			add_member(:public, Types::Struct.new(self,*args))
		end
		def public() ; @protection = :public ; end 
		def protected() ; @protection = :protected ; end 
		def private() ; @protection = :private ; end 
		def self.type_name() ; "class" ; end
	end
	class PreprocessorIf < Type
		def initialize(*args)
			super(*args)
		end
		def add_member(member)
			@members.push(member)
			return member
		end
		def self.type_name() ; "#if" ; end
		def open(state)
			state.needs_semi = false
			state.print("\n#if #{@name}")
		end
		def close(state)
			state.needs_semi = false
			state.print("\n#endif  // #{@name}")
		end
	end
	class Struct < Type
		attr_accessor :members
		def add_member(member)
			@members.push(member)
			return member
		end
		def self.type_name() ; "struct" ; end
	end
	class TemplateClass < Class
		def initialize(*args)
			super(*args)
			@t_args = CPP::Args.new()
		end
		def spec_args()
			return @spec_args ||= CPP::Args.new()
		end
		def template_header(state)
			if(@t_args.length > 0)
				state.pp("template < ")
				state.pp(@t_args)
			else
				state.pp("template < ")
			end
			state.pp(">\n")
		end
		def plan_cc(plan)

		end
		def plan_header(plan)
			group_plan = GroupPlan.new(self)
			plan.push(group_plan)
			@members.each do |member|
				group_plan.implicit = ProtectionGroups[@protections[member]]
				if(member.respond_to?(:plan_header))
					member.plan_header(group_plan)
				else
					group_plan.push(MemberElementInline.new(member))
				end
			end
		end
		def template_spec_args(state)
			if(@spec_args)
				state.pp("< ")
				state.pp(@spec_args)
				state.pp(">")
			end
		end
	end
end
class DepFilePair
	class NameSpace
		def initialize(name,parent)
			@name,@parent = name,parent
			@members = []
		end
		def add_struct(*args)
			add Types::Struct.new(self,*args)
		end
		def add_template(*args)
			add Types::TemplateClass.new(self,*args)
		end
		def add_class(*args)
			add Types::Class.new(self,*args)
		end
		def add_cc(arg)
			@members.push(CC_Mask.new(arg))
		end
		def chop_method_prefix()
			""
		end
		def class_comment(*args)
			add CPP::Comment.new(*args)
		end
		def var_dec_comment(*args)
			add CPP::Comment.new(*args)
		end
		def add_var_dec(arg)
			add DepMask.new(arg)
		end
		def plan_cc(plan)
			plan.implicit = ImplicitName.new(@name,plan.implicit)
			@members.each do |member|
				if(member.respond_to?(:plan_cc))
					member.plan_cc(plan)
				else
					MemberElementCC.check(plan,member)
				end
			end
			plan.implicit = plan.implicit.parent
		end
		def plan_header(plan)
			plan.implicit = ImplicitName.new(@name,plan.implicit)
			@members.each do |member|
				if(member.respond_to?(:plan_header))
					member.plan_header(plan)
				else
					MemberElementHeader.check(plan,member)
				end
			end
			plan.implicit = plan.implicit.parent
		end
		def add val
			@members << val
			val
		end
	end
	def initialize(rel_path)
		@rel_path = rel_path
		@cc_includes = []
		@header_includes = []
		@spaces = []
		@cc_usings = []
		@cache = {}
	end
	def add_cc_include(inc_path)
		@cc_includes << CPP::Include.new(inc_path)
	end
	def add_cc_using(using)
		@cc_usings << CPP::Using.new(using)
	end
	def add_header_include(inc_path)
		@header_includes << CPP::Include.new(inc_path)
	end
	def add_namespace(name,parent)
		space = NameSpace.new(name,parent)
		if(parent != self)
			parent.add(space)
		else
			@spaces.push(space)
		end
		return space
	end
	def set(type,cached)
		@cache[type] = cached
	end
	def get(type)
		cached = @cache[type]
		return cached if cached
		cached = type.create(self)
		cached_check = @cache[type]
		return cached_check if cached_check
		set(type,cached)
		return cached
	end
	def write_cc_file(cpp_base,cc_file)
		plan_cc = GroupPlan.new(nil,elems_cc = [])
		@spaces.each do |space|
			space.plan_cc(plan_cc)
		end
		suite = CPP::Suite.new(@cc_includes + @cc_usings + [plan_cc])
		CPP.pretty_print(suite,cc_file)
	end
	def write_header_file(cpp_base,header_file)
		plan_header = GroupPlan.new(nil,elems_cc = [])
		@spaces.each do |space|
			space.plan_header(plan_header)
		end
		suite = CPP::Suite.new(@header_includes + [plan_header])
		include_guard = CPP::IncludeGuard.new(suite)
		include_guard.name = @rel_path.upcase.gsub(/[\.\/\\]/,"_") + "_H_"
		CPP.pretty_print(include_guard,header_file)
	end
end
