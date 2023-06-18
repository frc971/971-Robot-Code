class SClipBBox
	attr_accessor :x, :y, :w, :h
	def initialize(x, y, w, h) @x = x; @y = y; @w = w; @h = h; end
	def ex() @x + @w end
	def ey() @y + @h end
	def ex=(v) @x = v - @w end
	def ey=(v) @y = v - @h end
	def sy=(y) @h += (@y - y); @y = y end
	def sx=(x) @w += (@x - x); @x = x end
	def hy=(y) @h = (y - @y) end
	def wx=(x) @w = (x - @x) end
	def contain(x, y)
		(@x <= x && @y <= y && x < @x + @w && y < @y + @h)
	end
	def enter(cr, &blk)
		cr.save()
		cr.rectangle(@x, @y, @w, @h)
		cr.translate(@x, @y)
		cr.clip()
		clipr = cr.get_clip_rectangle()[1]
		blk.call() if (clipr.width != 0 && clipr.height != 0)
		cr.restore()
		return self
	end
	InvalidBBox = SClipBBox.new(0,0,0,0)
end
