begin
  require "sha1"
rescue LoadError
  require "digest/sha1"
end

class Target::StructBase < Target::Node
  def create_DoGetType(type_class, cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"const ::aos::MessageType*","DoGetType")
    member_func.static = true
    fields = []
    register_members = []
    @members.each do |member|
      tId = member.getTypeID()
      fieldName = member.name.inspect
      if(member.respond_to?(:add_TypeRegister))
        register_members.push(member)
      end
      fields << "new ::aos::MessageType::Field{#{tId}, #{member.respond_to?(:length) ? member.length : 0}, #{fieldName}}"
    end
    register_members.uniq do |member|
      member.type
    end.each do |member|
      member.add_TypeRegister(cpp_tree, type_class, member_func)
    end
    id = getTypeID()
    member_func.suite << ("static const ::aos::MessageType kMsgMessageType(#{type_class.parent_class ? type_class.parent_class + '::Size()' : 0}, #{id}, #{(@loc.respond_to?(:queue_name) ? @loc.queue_name(@name) : "#{@loc.get_name()}.#{@name}").inspect}, {" +
                          "#{fields.join(", ")}})");
                          type_class.add_member(member_func)
                          member_func.suite << "::aos::type_cache::Add(kMsgMessageType)"
                          member_func.suite << CPP::Return.new("&kMsgMessageType")
  end
  def create_InOrderConstructor(type_class, cpp_tree)
    if @members.empty?
      return
    end
    cons = CPP::Constructor.new(type_class)
    type_class.add_member(cons)
    @members.each do |member|
      if member.respond_to?(:type_name)
        type_name = "#{member.type_name(cpp_tree)} #{member.name}_in"
      else
        type_name = member.create_usage(cpp_tree, "#{member.name}_in")
      end

      cons.args << type_name
      cons.add_cons(member.name, member.name + '_in')
    end
  end
  def create_DefaultConstructor(type_class, cpp_tree)
    cons = CPP::Constructor.new(type_class)
    type_class.add_member(cons)
    cons.add_cons(type_class.parent_class) if type_class.parent_class
    cons.suite << CPP::FuncCall.build('Zero')
  end
  def create_Zero(type_class,cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"void","Zero")
    type_class.add_member(member_func)
    @members.each do |elem|
      elem.zeroCall(member_func.suite)
    end
    member_func.suite << CPP::FuncCall.new(type_class.parent_class + '::Zero') if type_class.parent_class
  end
  def create_Size(type_class,cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"size_t","Size")
    member_func.inline = true
    member_func.static = true
    type_class.add_member(member_func.forward_dec)
    size = 0
    @members.each do |elem|
      size += elem.size
    end
    if type_class.parent_class
      member_func.suite << CPP::Return.new(CPP::Add.new(size,
                                                        "#{type_class.parent_class}::Size()"))
    else
      member_func.suite << CPP::Return.new(size)
    end
  end
  def create_Serialize(type_class,cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"size_t","Serialize")
    type_class.add_member(member_func)
    member_func.args << "char *buffer"
    member_func.suite << "#{type_class.parent_class}::Serialize(buffer)" if type_class.parent_class
    member_func.const = true
    offset = type_class.parent_class ? type_class.parent_class + '::Size()' : '0'
    @members.each do |elem|
      elem.toNetwork(offset,member_func.suite)
      offset += " + #{elem.size}";
    end
    member_func.suite << CPP::Return.new(CPP::FuncCall.new('Size'))
  end
  def create_Deserialize(type_class,cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"size_t","Deserialize")
    type_class.add_member(member_func)
    member_func.args << "const char *buffer"
    member_func.suite << "#{type_class.parent_class}::Deserialize(buffer)" if type_class.parent_class
    offset = type_class.parent_class ? type_class.parent_class + '::Size()' : '0'
    @members.each do |elem|
      elem.toHost(offset,member_func.suite)
      offset += " + #{elem.size}";
    end
    member_func.suite << CPP::Return.new(CPP::FuncCall.new('Size'))
  end
  def create_EqualsNoTime(type_class,cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"bool","EqualsNoTime")
    member_func.const = true
    type_class.add_member(member_func)
    member_func.args << "const #{type_class.name} &other"
    member_func.suite << "if (!#{type_class.parent_class}::EqualsNoTime(other)) return false;" if type_class.parent_class
    @members.each do |elem|
      if elem.respond_to? :create_EqualsNoTime
        member_func.suite << "if (!other.#{elem.name}.EqualsNoTime(#{elem.name})) return false;"
      elsif elem.respond_to?(:length) && elem.member_type.respond_to?(:create_EqualsNoTime)
        0.upto(elem.length - 1) do |i|
          member_func.suite << "if (!other.#{elem.name}[#{i}].EqualsNoTime(#{elem.name}[#{i}])) return false;"
        end
      else
        member_func.suite << "if (other.#{elem.name} != #{elem.name}) return false;"
      end
    end
    member_func.suite << CPP::Return.new('true')
  end
  def simpleStr()
    return "{\n" + @members.collect() { |elem| elem.simpleStr() + "\n"}.join("") + "}"
  end
  def getTypeID()
    return "0x" + (((Digest::SHA1.hexdigest(simpleStr())[0..3].to_i(16)) << 16) | size).to_s(16)
  end
  def add_member(member)
    @members << member
  end
  def size()
    return @size if(@size)
    @size = 0
    @members.each do |elem|
      @size += elem.size
    end
    return @size
  end
end

class Target::MessageDec < Target::StructBase
  attr_accessor :name,:loc,:parent,:msg_hash
  def initialize(name)
    @name = name
    @members = []
  end
  def extern=(value)
    @extern=value
  end
  def get_name()
    if(@parent)
      return "#{@parent.get_name}.#{@name}"
    else
      return "#{@name}"
    end
  end
  def create_Print(type_class,cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"size_t","Print")
    type_class.add_member(member_func)
    member_func.args << "char *buffer"
    member_func.args << "size_t length"
    member_func.const = true
    format = "\""
    args = []
    @members.each do |elem|
      format += ", "
      format += elem.toPrintFormat()
      elem.fetchPrintArgs(args)
    end
    format += "\""
    member_func.suite << "size_t super_size = ::aos::Message::Print(buffer, length)"
    member_func.suite << "buffer += super_size"
    member_func.suite << "length -= super_size"
    if !args.empty?
      member_func.suite << "return super_size + snprintf(buffer, length, " + ([format] + args).join(", ") + ")";
    else
      # snprintf will return zero.
      member_func.suite << "return super_size"
    end
  end
  def create_GetType(type_class, cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"const ::aos::MessageType*","GetType")
    type_class.add_member(member_func)
    member_func.static = true
    member_func.suite << "static ::aos::Once<const ::aos::MessageType> getter(#{type_class.name}::DoGetType)"
    member_func.suite << CPP::Return.new("getter.Get()")
  end
  def self.builder_loc(loc)
    return @builder_loc if(@builder_loc)
    return @builder_loc = loc.root.get_make("aos")
  end
  def type_name(builder_loc,name)
    if(builder_loc == @loc) #use relative name
      return name
    else #use full name
      return @loc.to_cpp_id(name)
    end
  end
  def create(cpp_tree)
    return self if(@extern)
    orig_namespace = namespace = cpp_tree.get(@loc)
    name = ""
    if(namespace.class < Types::Type) #is nested
      name = namespace.name + "_" + name
      namespace = namespace.space
    end
    type_class = namespace.add_struct(name + @name)
    if(name.length > 0)
      orig_namespace.add_member(:public, Types::TypeDef.new(type_class,@name))
    end
    cpp_tree.set(self,type_class)
    type_class.set_parent("public ::aos::Message")
    ts = self.simpleStr()
    self.msg_hash = "0x#{Digest::SHA1.hexdigest(ts)[-8..-1]}"
    type_class.add_member("enum {kQueueLength = 100, kHash = #{self.msg_hash}}")
    @members.each do |elem|
      type_class.add_member(elem.create_usage(cpp_tree))
    end

    create_Serialize(type_class,cpp_tree)
    create_Deserialize(type_class,cpp_tree)
    create_Zero(type_class,cpp_tree)
    create_Size(type_class,cpp_tree)
    create_Print(type_class,cpp_tree)
    create_GetType(type_class, cpp_tree)
    create_DoGetType(type_class, cpp_tree)
    create_DefaultConstructor(type_class, cpp_tree)
    create_InOrderConstructor(type_class, cpp_tree)
    create_EqualsNoTime(type_class, cpp_tree)

    b_namespace = cpp_tree.get(b_loc = self.class.builder_loc(@loc))

    template = b_namespace.add_template("MessageBuilder")
    template.spec_args << t = @loc.to_cpp_id(@name)
    msg_ptr_t = "ScopedMessagePtr< #{t}>"
    msg_bld_t = "MessageBuilder< #{t}>"
    template.add_member(:private,"#{msg_ptr_t} msg_ptr_")
    template.add_member(:private,"#{msg_bld_t}(const #{msg_bld_t}&)")
    template.add_member(:private,"void operator=(const #{msg_bld_t}&)")
    template.add_member(:private,"friend class ::aos::Queue< #{t}>")

    cons = CPP::Constructor.new(template)
    template.add_member(:private,cons)
    cons.args << "RawQueue *queue"
    cons.args << "#{t} *msg"
    cons.add_cons("msg_ptr_","queue","msg")
    template.public
    DefineMembers(cpp_tree, template, msg_bld_t)
  end
  def DefineMembers(cpp_tree, template, msg_bld_t) 
    send = template.def_func("bool","Send")
    send.suite << CPP::Return.new("msg_ptr_.Send()")
    @members.each do |elem|
=begin
      MessageBuilder<frc971::control_loops::Drivetrain::Goal> &steering(
              double steering) {
            msg_ptr_->steering = steering;
              return *this;
              }
=end
      setter = template.def_func(msg_bld_t,"&#{elem.name}")
        setter.args << elem.create_usage(cpp_tree)
      elem.set_message_builder(setter.suite)
      setter.suite << CPP::Return.new("*this")

    end
  end
end
class Target::MessageElement < Target::Node
  attr_accessor :name,:loc,:size,:zero,:type,:printformat
  def initialize(type,name)
    @type,@name = type,name
  end
  def type()
    return @type
  end
  def type_name(cpp_tree)
    return type()
  end
  def toPrintFormat()
    return printformat
  end
  def create_usage(cpp_tree, this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    "#{@type} #{this_name}"
  end
  def toNetwork(offset,suite, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    suite << f_call = CPP::FuncCall.build("to_network",
                                          "&#{parent}#{this_name}",
                                          "&buffer[#{offset}]")
    f_call.args.dont_wrap = true
  end
  def toHost(offset,suite, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    suite << f_call = CPP::FuncCall.build("to_host",
                                          "&buffer[#{offset}]",
                                          "&#{parent}#{this_name}")
                                            f_call.args.dont_wrap = true
  end
  def getTypeID()
    '0x' + ((Digest::SHA1.hexdigest(@type)[0..3].to_i(16) << 16) |
            0x2000 | # marks it as primitive
            size).to_s(16)
  end
  def simpleStr()
    "#{@type} #{@name}"
  end
  def set_message_builder(suite, this_name=nil)
    this_name ||= @name
    suite << "msg_ptr_->#{this_name} = #{this_name}"
  end

  def zeroCall(suite, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    suite << CPP::Assign.new(parent + this_name,@zero)
  end
  def fetchPrintArgs(args, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    if (@type == 'bool')
      args.push("#{parent}#{this_name} ? 'T' : 'f'")
    elsif (@type == '::aos::time::Time')
      args.push("AOS_TIME_ARGS(#{parent}#{this_name}.sec(), #{parent}#{this_name}.nsec())")
    else
      args.push("#{parent}#{this_name}")
    end
  end
end
class Target::MessageArrayElement < Target::Node
  attr_accessor :loc, :length
  def initialize(type,length)
    @type,@length = type,length
  end
  def zero()
    return @type.zero()
  end
  def name()
    return @type.name()
  end

  def add_TypeRegister(*args)
    if @type.respond_to?(:add_TypeRegister)
      @type.add_TypeRegister(*args)
    end
  end

  def toPrintFormat()
    return ([@type.toPrintFormat] * @length).join(", ")
  end
  def create_usage(cpp_tree, this_name=nil)
    if (this_name == nil)
      this_name = name
    end
    return "::std::array< #{@type.type_name(cpp_tree)}, #{@length}> #{this_name}"
  end
  def type()
    return "::std::array< #{@type.type()}, #{@length}>"
  end
  def member_type()
    return @type
  end
  def simpleStr()
    return "#{@type.type} #{@name}[#{@length}]"
  end
  def getTypeID()
    return @type.getTypeID()
  end
  def size()
    return @type.size * @length
  end
  def toNetwork(offset, suite, parent="", this_name=nil)
    if (this_name == nil)
      this_name = name
    end
    offset = (offset == 0) ? "" : "#{offset} + "
    for_loop_var = getForLoopVar()
    suite << for_stmt = CPP::For.new("int #{for_loop_var} = 0","#{for_loop_var} < #{@length}","#{for_loop_var}++")
    @type.toNetwork("#{offset} (#{for_loop_var} * #{@type.size})", for_stmt.suite, parent, "#{this_name}[#{for_loop_var}]")
  end
  def toHost(offset, suite, parent="", this_name=nil)
    if (this_name == nil)
      this_name = name
    end
    offset = (offset == 0) ? "" : "#{offset} + "
    for_loop_var = getForLoopVar()
    suite << for_stmt = CPP::For.new("int #{for_loop_var} = 0","#{for_loop_var} < #{@length}","#{for_loop_var}++")
    @type.toHost("#{offset} (#{for_loop_var} * #{@type.size})", for_stmt.suite, parent, "#{this_name}[#{for_loop_var}]")
  end
  def set_message_builder(suite, this_name=nil)
    if (this_name == nil)
      this_name = name
    end

    for_loop_var = getForLoopVar()
    suite << for_stmt = CPP::For.new("int #{for_loop_var} = 0","#{for_loop_var} < #{@length}","#{for_loop_var}++")
    @type.set_message_builder(for_stmt.suite, "#{this_name}[#{for_loop_var}]")
  end
  def zeroCall(suite, parent="", this_name=nil)
    if (this_name == nil)
      this_name = name
    end

    offset = (offset == 0) ? "" : "#{offset} + "
    for_loop_var = getForLoopVar()
    suite << for_stmt = CPP::For.new("int #{for_loop_var} = 0","#{for_loop_var} < #{@length}","#{for_loop_var}++")
    @type.zeroCall(for_stmt.suite, parent, "#{this_name}[#{for_loop_var}]")
  end
  def fetchPrintArgs(args, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = name
    end
    for i in 0..(@length-1)
      @type.fetchPrintArgs(args, parent, "#{this_name}[#{i}]")
    end
  end
end
