#system("bazel build -c opt //geometry/voronoi:test.so")

require "tokens/tokenizer_lib_base.rb"
#require "geometry/voronoi/test.so"
class CompGeo
end

class CompGeo::Point2di
	init_fields :x, :y
	def self.make(a, b)
		CompGeo::Point2di.new(a, b)
	end
	def +(o) CompGeo::Point2di.new(@x + o.x, @y + o.y) end
	def -(o) CompGeo::Point2di.new(@x - o.x, @y - o.y) end
	def /(o) CompGeo::Point2di.new(@x / o, @y / o) end
	def *(o) CompGeo::Point2di.new(@x * o, @y * o) end
	def mag_sqr() return @x * @x + @y * @y end
	def xf() @x.to_f end
	def yf() @y.to_f end
	def to_i_vec() CompGeo::Point2di.new(@x.to_i, @y.to_i) end
	def round_up_fudge() CompGeo::Point2di.new(@x <=> 0, @y <=> 0) end
	def dot(o) @x * o.x + @y * o.y end
#	alias :old_cross :cross
	def cross(o)
		v = @x * o.y - @y * o.x
		#v2 = old_cross(o)
		#raise "problem! #{v} #{v2}" if (v != v2)
		return v
	end
	def ==(o) self.class == o.class && @x == o.x && @y == o.y end
	
	def dist(o)
		return ArcPointTime.new((self - o).mag_sqr(), 0)
	end
	def rot90()
		return CompGeo::Point2di.new(-@y, @x) 
	end
end
Point2di = CompGeo::Point2di

def integer_sqrt(n)
	n = n * 4
	i = n >> (n.bit_length / 2)
	oi = n 
	k = 0
	while (oi / 2 != i / 2 && k < 100)
		oi = i
		i = (n / i + i) / 2
		k += 1
	end
	raise "integer sqrt did not converge #{n}" if k == 100
	return i / 2
end
