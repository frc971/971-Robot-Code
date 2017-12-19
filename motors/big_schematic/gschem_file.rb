VersionRegex = /v (\d{8}) (\d+)/
ComponentRegex = /C (\d+) (\d+) ([01]) (\d{1,3}) ([01]) (.+)/
TextRegex = /T (\d+) (\d+) (\d+) (\d+) ([01]) ([012]) (\d{1,3}) ([0-8]) (\d+)/
PowerRegex = /^(([0-9.]+V-plus)|(title-.)|(gnd)|(output)|(input)|(io)|(vcc)|-1)|(vbat)|(vhalfbat)|(generic-power).sym$/

# All coordinates are in "mils".

class Component
  attr_accessor :x, :y, :selectable, :angle, :mirrored, :filename
  attr_accessor :attributes

  def initialize(x, y, selectable, angle, mirrored, filename)
    @x, @y, @selectable = x, y, selectable
    @angle, @mirrored, @filename = angle, mirrored, filename

    @attributes = {}
  end

  def refdes
    if attributes.has_key?(:refdes)
      attributes[:refdes].value
    else
      nil
    end
  end

  def [](key)
    r = attributes[key]
    if r
      r.value
    else
      nil
    end
  end

  def is_power
    filename =~ PowerRegex
  end
end

def parse_bool(string)
  if string == '0'
    return false
  elsif string == '1'
    return true
  else
    raise "'#{string}' is not a bool"
  end
end

class Attribute
  attr_accessor :x, :y, :color, :size, :visible, :show, :angle, :alignment
  attr_accessor :name, :value

  def initialize(x, y, color, size, visible, show, angle, alignment, name, value)
    @x, @y, @color, @size, @visible, @show = x, y, color, size, visible, show
    @angle, @alignment, @name, @value = angle, alignment, name, value
  end

  def Attribute.parse(match, value_line)
    raise "Don't know what multi-line attributes mean" if match[9] != '1'
    attribute_value = value_line.chomp.split('=', 2)

    Attribute.new(match[1].to_i,
                  match[2].to_i,
                  match[3].to_i,
                  match[4].to_i,
                  parse_bool(match[5]),
                  match[6],
                  match[7].to_i,
                  match[8].to_i,
                  attribute_value[0],
                  attribute_value[1])
  end
end

class GschemFile
  attr_accessor :version_date, :version

  def initialize(filename)
    lines = nil
    File.open(filename) do |f|
      lines = f.readlines
    end
    version_line = lines.shift
    version_match = VersionRegex.match version_line
    raise "Can't parse version line '#{version_line}'" unless version_match
    @version_date = version_match[1]
    @version = version_match[2].to_i

    return lines
  end
end

class GschemSymbol < GschemFile
  attr_accessor :attributes

  def initialize(filename)
    lines = super(filename)

    @attributes = {}

    until lines.empty?
      line = lines.shift

      match = TextRegex.match(line)
      if match
        attribute = Attribute.parse(match, lines.shift)
        @attributes[attribute.name.intern] = attribute
      end
    end
  end

  def [](key)
    r = attributes[key]
    if r
      r.value
    else
      nil
    end
  end
end

class GschemSchematic < GschemFile
  attr_accessor :components

  def initialize(filename)
    lines = super(filename)

    @components = []

    until lines.empty?
      line = lines.shift

      match = ComponentRegex.match(line)
      if match
        c = Component.new(match[1].to_i,
                          match[2].to_i,
                          parse_bool(match[3]),
                          match[4].to_i,
                          parse_bool(match[5]),
                          match[6])
        line = lines.shift
        unless line
          # There's a component without {} at the end of the file.
          raise "Huh?" unless lines.empty?
          next
        end
        if line.chomp == '{'
          line = lines.shift
          until line.chomp == '}'
            match = TextRegex.match(line)
            raise "Error parsing attribute '#{line}'" unless match

            attribute = Attribute.parse(match, lines.shift)
            c.attributes[attribute.name.intern] = attribute

            line = lines.shift
          end
        else
          lines.unshift line
        end
        @components.push c
        next
      end
    end
  end
end

def get_schematic_filenames files
  r = []
  files.each do |file|
    if file.end_with? '.gsch2pcb'
      File.open(file, 'r') do |f|
        dirname = File.dirname(file) + '/'
        r += f.readline.split(' ')[1..-1].map do |filename|
          dirname + filename
        end
      end
    else
      r.push file
    end
  end
  r
end
