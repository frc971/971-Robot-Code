class Color
	def initialize(r, g, b)
		@r, @g, @b = r, g, b
	end 
	def set(cr) 
		cr.set_source_rgb(@r, @g, @b) 
	end 
	def hex_code()
		r = (@r * 255).to_i.to_s(16).rjust(2, "0")
		g = (@g * 255).to_i.to_s(16).rjust(2, "0")
		b = (@b * 255).to_i.to_s(16).rjust(2, "0")
		"##{r}#{g}#{b}"
	end 
	def code_255()
		return 16 + 36*(@r * 5).to_i + 6*(@g * 5).to_i + (@b * 5).to_i
	end 
	def pattr_fg(st, ed)
		attr = Pango::AttrForeground.new(65535 * @r, 65535 * @g, 65535 * @b)
		attr.start_index = st
		attr.end_index = ed
		return attr
	end
	def pattr_bg(st, ed)
		attr = Pango::AttrBackground.new(65535 * @r, 65535 * @g, 65535 * @b)
		attr.start_index = st
		attr.end_index = ed
		return attr
	end
	Yellow = self.new(1.0, 1.0, 0.0)
	Red = self.new(1.0, 0.0, 0.0)
	Green = self.new(0.0, 1.0, 0.0)
	Blue = self.new(0.0, 0.0, 1.0)
	Pink = self.new(1.0, 0.5, 0.5)
	Gray = self.new(0.5, 0.5, 0.5)
	Teal = self.new(0.5, 0.5, 1.0)
	Black = self.new(0.0, 0.0, 0.0)
	White = self.new(1.0, 1.0, 1.0)
end
