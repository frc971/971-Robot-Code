class Point2d
	attr_accessor :x, :y
	def initialize(x, y) @x, @y = x, y end
	def inspect() "<#{@x}, #{@y}>" end
	def to_s() "#<Point2d:#{@x} #{@y}>" end
	def +(other) Point2d.new(@x + other.x, @y + other.y) end
	def -(other) Point2d.new(@x - other.x, @y - other.y) end
	def scale(d)
		return Point2d.new(@x * d, @y * d)
	end
	def -@()
		return Point2d.new(-@x, -@y)
	end
	def dot(o)
		return o.x * @x + o.y * @y
	end
	def area(o)
		return o.x * @y - o.y * @x
	end
	def normalize()
		d = Math.sqrt(@x * @x + @y * @y)
		return Point2d.new(@x / d, @y / d)
	end
	def mag() Math.sqrt(@x * @x + @y * @y) end
	def mag_sqr() @x * @x + @y * @y end
	def div(d) Point2d.new(@x / d, @y / d) end

	def comp_h(o)
		a = @x <=> o.x
		return a if a != 0
		return @y <=> o.y
	end
	def comp_v(o)
		a = @y <=> o.y
		return a if a != 0
		return @x <=> o.x
	end
	def rot90()
		return Point2d.new(-@y, @x)
	end
end
