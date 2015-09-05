class Target::StructDec < Target::StructBase
  attr_accessor :name,:loc,:parent, :extern
  def initialize(name)
    @name = name
    @members = []
  end
  def add_member(member)
    @members << member
  end
  def create_GetType(type_class, cpp_tree)
    member_func = CPP::MemberFunc.new(type_class,"const ::aos::MessageType*","GetType")
    type_class.add_member(member_func)
    member_func.static = true
    member_func.suite << "static ::aos::Once<const ::aos::MessageType> getter(#{type_class.name}::DoGetType)"
    member_func.suite << CPP::Return.new("getter.Get()")
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

    @members.each do |elem|
      type_class.add_member(elem.create_usage(cpp_tree))
    end
    create_DoGetType(type_class, cpp_tree)
    create_GetType(type_class, cpp_tree)
    create_DefaultConstructor(type_class, cpp_tree)
    create_InOrderConstructor(type_class, cpp_tree)
    create_Zero(type_class, cpp_tree)
    create_Size(type_class, cpp_tree)
    create_Serialize(type_class, cpp_tree)
    create_Deserialize(type_class, cpp_tree)
    create_EqualsNoTime(type_class, cpp_tree)
    return type_class
  end
  def getPrintFormat()
    return "{" + @members.collect { |elem| elem.toPrintFormat() }.join(", ") + "}"
  end
  def fetchPrintArgs(args, parent = "")
    @members.each do |elem|
      elem.fetchPrintArgs(args, parent)
    end
  end
  def toHost(offset, suite, parent)
    @members.each do |elem|
      elem.toHost(offset, suite, parent)
      offset += " + #{elem.size()}"
    end
  end
  def toNetwork(offset, suite, parent)
    @members.each do |elem|
      elem.toNetwork(offset, suite, parent)
      offset += " + #{elem.size()}"
    end
  end
  def zeroCall(suite, parent)
    @members.each do |elem|
      elem.zeroCall(suite, parent)
    end
  end
end
class Target::MessageStructElement < Target::Node
  attr_accessor :name,:loc
  def initialize(type,name)
    @type, @name = type, name
  end
  def type()
    return @type.get_name
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
  def type_name(cpp_tree)
    type = cpp_tree.get(@type)
    if(@type.loc == @loc) #use relative name
      return type.name
    else #use full name
      return @type.loc.to_cpp_id(type.name)
    end 
  end
  def size()
    return @type.size()
  end
  def toPrintFormat()
    @type.getPrintFormat()
  end
  def create_usage(cpp_tree, this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    return "#{type_name(cpp_tree)} #{this_name}"
  end
  def add_TypeRegister(cpp_tree, o_type, member_func)
    type = cpp_tree.get(@type)
    tName = @type.loc.to_cpp_id(type.name)
    member_func.suite << "#{tName}::GetType()"
  end
  def fetchPrintArgs(args, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    @type.fetchPrintArgs(args, parent + "#{this_name}.")
  end
  def toNetwork(offset,suite, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    @type.toNetwork(offset, suite, parent + "#{this_name}.")
  end
  def toHost(offset,suite, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    @type.toHost(offset, suite, parent + "#{this_name}.")
  end
  def set_message_builder(suite, this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    suite << "msg_ptr_->#{this_name} = #{this_name}"
  end

  def zeroCall(suite, parent = "", this_name=nil)
    if (this_name == nil)
      this_name = @name
    end
    @type.zeroCall(suite, parent + "#{this_name}.")
  end
  def simpleStr()
    "#{@type.simpleStr()} #{@name}"
  end
  def getTypeID()
    return @type.getTypeID()
  end
end
