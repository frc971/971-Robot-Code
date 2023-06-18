class DXFParser
	class Section
		attr_accessor :sec_name
		def initialize(sec_name)
			@sec_name = sec_name
			@items = []
		end

		def print_sec()
			puts "section #{@sec_name}:"
			@items.each do |item|
				puts "  #{item.inspect}"
			end
		end

		def add_item(item)
			@items << item
		end

		def self.make_section(sec_name)
			return if sec_name != "ENTITIES"
			return Section.new(sec_name)
		end
	end
	class GroupPair
		attr_accessor :group_code, :value
		def initialize(file)
			@group_code = file.gets().match(/\d+/)[0].to_i
			@value = file.gets().chomp()
			if (@group_code == 0)
				@value.upcase!
			end
#		rescue
#			raise Exception.new("dxf file does not follow expected format.")
		end
		def inspect()
			"#{@group_code}: #{@value.inspect}"
		end
	end
	def initialize(fname)
		@file = fname.class == File ? fname : File.open(fname, "r")
		@seen_eof = false
	end
	def get_next()
		return GroupPair.new(@file)
	end
	def expect_next(code)
		obj = get_next()
		raise "expected group_code: #{code} not #{obj.inspect}" if obj.group_code != code
		return obj
	end
	def get_section(section_parser = Section)
		while true
			return if @seen_eof
			obj = expect_next(0)
			if obj.value == "EOF"
				@seen_eof = true
				return
			end
			raise "expected SECTION\n" if (obj.value != "SECTION")
			secname = expect_next(2).value
			secval = section_parser.make_section(secname)
			while true
				obj = get_next()
				if (obj.group_code == 0 && obj.value == "ENDSEC")
					if secval
						secval.finished_parsing() if secval.respond_to?(:finished_parsing)
						return secval
					end
					break
				end
				secval.add_item(obj) if secval
			end
		end
	end
	class SectionRep
		def self.set_section_type(section_type) @section_type = section_type end
		def self.section_type() @section_type end
		def self.make_section(section)
			return self.new()
		end
	end
	class Entities < SectionRep
		set_section_type "ENTITIES"
		def initialize()
		end
		def add_item(item)
			if (item.group_code == 0)
			end
		end
	end
	class EntityParse
		class Instance
			def initialize(entity_parse)
				@entity_parse = entity_parse
				@items = []
			end
			attr_accessor :items
			def add_item(item)
				if (item.group_code == 0)
					@items.push(@last_item) if @last_item
					@last_item = nil
					@item_parse = @entity_parse.item_parser(item.value)
					@last_item = @item_parse.start_item() if @item_parse
				else
					@item_parse.handle(item, @last_item) if(@item_parse)
				end
			end
			def print_sec()
				puts "section ENTITIES:"
				@items.each do |item|
					puts item.inspect
				end
			end
		end
		class ItemParser
			def initialize(item_type) @item_type = item_type
				@handlers = {}
			end
			def add(group_code, &parse_block)
				@handlers[group_code] = parse_block
			end
			def handle(item, obj)
				handler = @handlers[item.group_code]
				if (handler)
					handler.call(obj, item.value)
				elsif obj.respond_to?(:extras)
					obj.extras(item)
				end
			end
			def start_item()
				return @item_type.new()
			end
		end
		def section_type() return "ENTITIES" end
		def initialize()
			@item_parsers = {}
		end
		def add_item_type(key, parser)
			@item_parsers[key] = ItemParser.new(parser)
		end
		def item_parser(key)
			return @item_parsers[key]
		end
		def make_section(section)
			return Instance.new(self)
		end
	end
	class DelagateParser
		def initialize(parser_list)
			@per_section = {}
			parser_list.each do |parser|
				@per_section[parser.section_type()] = parser
			end
		end
		def make_section(section)
			return if !(parse = @per_section[section])
			return parse.make_section(section)
		end
	end
	JustEntities = DelagateParser.new([Entities])
end

class Point3d
	attr_accessor :x, :y, :z
	def initialize(x, y, z) @x, @y, @z = x, y, z end
	def inspect() "<#{@x}, #{@y}, #{@z}>" end
	def project() Point2d.new(@x, @y) end
end
class Line
	attr_accessor :st, :ed
	def initialize()
		@st = Point3d.new(0.0, 0.0, 0.0)
		@ed = Point3d.new(0.0, 0.0, 0.0)
	end
	def draw_to(cr, scale)
		cr.move_to(@st.x * scale, - @st.y * scale)
		cr.line_to(@ed.x * scale, - @ed.y * scale)
		cr.stroke()
	end
	def corners() return [@st.project, @ed.project] end
	def to_2d() return Line2d.new(@st.project, @ed.project) end
end
class Arc
	attr_accessor :c, :r, :st, :ed
	def initialize()
		@c = Point3d.new(0.0, 0.0, 0.0)
	end
	def draw_to(cr, scale)
		cr.arc(@c.x * scale, - @c.y * scale, @r * scale, -@ed * Math::PI / 180.0, -@st * Math::PI / 180.0)
		cr.stroke()
	end
	def corners()
		c = @c.project()
		st_theta = @st * Math::PI / 180.0
		ed_theta = @ed * Math::PI / 180.0
		[
		 c + Point2d.new(@r * Math.cos(st_theta), @r * Math.sin(st_theta)),
		 c + Point2d.new(@r * Math.cos(ed_theta), @r * Math.sin(ed_theta)),
		]
	end
	def to_2d() return Arc2d.new(@c.project, @r, @st / 180.0 * Math::PI, @ed / 180.0 * Math::PI) end
end
class Circle
	attr_accessor :c, :r
	def initialize()
		@c = Point3d.new(0.0, 0.0, 0.0)
	end
	def draw_to(cr, scale)
		cr.arc(@c.x * scale, - @c.y * scale, @r * scale, 0, Math::PI * 2)
		cr.stroke()
	end
	def corners() return [] end
	def to_2d() return CircleProfile2d.new(@c.project, @r) end
end
def get_entities(filename)
	entity_parse = DXFParser::EntityParse.new()
	line_parse = entity_parse.add_item_type("LINE", Line)
	line_parse.add(10) { |l, v| l.st.x = v.to_f}
	line_parse.add(20) { |l, v| l.st.y = v.to_f}
	line_parse.add(30) { |l, v| l.st.z = v.to_f}
	line_parse.add(11) { |l, v| l.ed.x = v.to_f}
	line_parse.add(21) { |l, v| l.ed.y = v.to_f}
	line_parse.add(31) { |l, v| l.ed.z = v.to_f}

	line_parse = entity_parse.add_item_type("ARC", Arc)
	line_parse.add(10) { |l, v| l.c.x = v.to_f}
	line_parse.add(20) { |l, v| l.c.y = v.to_f}
	line_parse.add(30) { |l, v| l.c.z = v.to_f}
	line_parse.add(40) { |l, v| l.r = v.to_f}
	line_parse.add(50) { |l, v| l.st = v.to_f}
	line_parse.add(51) { |l, v| l.ed = v.to_f}

	line_parse = entity_parse.add_item_type("CIRCLE", Circle)
	line_parse.add(10) { |l, v| l.c.x = v.to_f}
	line_parse.add(20) { |l, v| l.c.y = v.to_f}
	line_parse.add(30) { |l, v| l.c.z = v.to_f}
	line_parse.add(40) { |l, v| l.r = v.to_f}

	parser = DXFParser::DelagateParser.new([entity_parse])
	dxf_file = DXFParser.new(filename)
	#dxf_file = DXFParser.new("flat_dxf.dxf") 

	sec = dxf_file.get_section(parser)

	raise "error! two entity sections" if dxf_file.get_section(parser) != nil
	return sec.items
end
