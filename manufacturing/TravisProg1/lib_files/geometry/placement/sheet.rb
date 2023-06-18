require "data/codecs/data_view.rb"
require "data/codecs/serialize_buffer.rb"
class SheetLayout
	class BBox
		attr_accessor :min, :max
		def initialize()
			reset()
		end
		def reset()
			inf = Float::INFINITY
			@min = Point2d.new(inf, inf)
			@max = Point2d.new(-inf, -inf)
		end
		def add_point(pt)
			@min.x = pt.x if pt.x < @min.x
			@min.y = pt.y if pt.y < @min.y
			@max.x = pt.x if pt.x > @max.x
			@max.y = pt.y if pt.y > @max.y
		end
	end
	class Pos
		attr_accessor :c, :rot, :item
		def initialize(c, rot, item)
			@c, @rot, @item = c, rot, item
			# @bbox = BBox.new()
		end
		def draw_to(cr, scale)
			cr.save()
			cr.translate(c.x * scale, -c.y * scale)
			cr.rotate(-@rot)
			@item.draw_to(cr, scale)
			cr.restore()
		end
		def query_crossings(q)
			q = q - @c
			c = Math.cos(-@rot)
			s = Math.sin(-@rot)
			q = Point2d.new(q.x * c - q.y * s, q.x * s + q.y * c)
			@item.query_crossings(q)
		end
		def update(delta)
			@c += delta
		end
		def rotate(cent, da)
			creal = cent
			d1 = @c - cent 
			c = Math.cos(-@rot)
			s = Math.sin(-@rot)
			d1 = Point2d.new(d1.x * c - d1.y * s, d1.x * s + d1.y * c)

			@rot += da

			c = Math.cos(@rot)
			s = Math.sin(@rot)
			dc = Point2d.new(c * d1.x - d1.y * s, d1.x * s + d1.y * c)
			@c = creal + dc
		end
	end

	def initialize()
		@items = []
		@selected = nil
	end

	attr_accessor :selected
	def delete(item) @items.delete(item) end

	def items() @items end

	def add_item(item, c, rot)
		@items << Pos.new(c, rot, item)
	end
	def pick(q)
		@selected = nil
		@items.each do |item|
			cp_count = item.query_crossings(q)
			return (@selected = item) if cp_count == 1
		end
		return nil
	end
	def draw_to(cr, scale, fixed = nil)
		@items.each do |item|
			(@selected == item ? Color::Red : Color::Green).set(cr)
			Color::Blue.set(cr) if (fixed)
			item.draw_to(cr, scale)
		end
	end
end

def draw_crosshair(cr, scale, pt)
	cr.move_to(pt.x * scale + 5, -pt.y * scale + 5)
	cr.line_to(pt.x * scale - 5, -pt.y * scale - 5)
	cr.stroke()
	cr.move_to(pt.x * scale + 5, -pt.y * scale - 5)
	cr.line_to(pt.x * scale - 5, -pt.y * scale + 5)
	cr.stroke()
end

class Point2d
	def write_to(ser)
		ser.write_f64(@x)
		ser.write_f64(@y)
	end
	def self.load_from(ser)
		self.new(ser.parse_f64(), ser.parse_f64())
	end
end

class SheetLayout
	class Pos
		def write_to(ser)
			ser.write_f64(@c.x)
			ser.write_f64(@c.y)
			ser.write_f64(@rot)
			ser.code_string(@item.filename)
		end
	end
	def write_to(ser)
		ser.push()
		@items.each do |item|
			item.write_to(ser)
		end
		ser.pop()
	end
	def self.load_file(fname, pm_loader)
		layout = self.new()
		reader = StreamReader.new(File.open(fname, "rb").read())
		reader.push()
		while (reader.not_done())
			pt = Point2d.load_from(reader)
			ang = reader.parse_f64()
			ofname = reader.parse_string()
			pm = pm_loader.load_pm(ofname)
			layout.add_item(pm, pt, ang)
			#puts "<#{pt.x}, #{pt.y}> #{ang} #{ofname.inspect}"
		end
		reader.pop()
		return layout
	end
end

def write_out_sheet(sheet, fname)
	ser = SerializeBuffer::Writer.new()
	sheet.write_to(ser)
	data = ser.data()

	len = SerializeBuffer.get_concat_length(data)
	buffer = OutputView.new(len)
	SerializeBuffer.concat_output(buffer, data)
	File.open(fname, "w+") { |f| f.write(buffer.val) }
end
