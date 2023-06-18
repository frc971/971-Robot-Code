require "pango"

require_relative "pango_tools.rb"

class Pango::Layout
	def inspect()
		ls = self.lines()
		txt = "pango_layout {\n"
		ls.each do |line|
			txt += "  #{line.display_line(font_description)}\n"
		end
		return txt + "}"
	end
end
class Pango::LayoutLine
  def inspect() "#<pango_line: #{display_line(layout.font_description)}>" end
	def display_line(desc)
		"[ #{runs.collect { |rn| rn.display_run(desc)}.join(", ")}]"
	end
end
FontTable = {}
def get_font_mapping(i)
	if (FontTable.length == 0)
		l = Gdk.default_root_window.create_cairo_context.create_pango_layout
		l.set_font_description(Desc)
		" ".upto("~") do |c|
			l.set_text(c)
			g = l.lines[0].runs[0].glyphs.glyphs[0][0].glyph
			puts g.inspect
			puts g - c.ord
			puts "#{g} => #{c.ord}"
			FontTable[g] = c
		end
		txt = " ".upto("~").to_a.join()
		l.set_text(txt)
		#puts c.inspect
	end
	return FontTable[i]
end
class Pango::GlyphItem
	def display_run(desc)
    throw Exception.new("Glyph item is buggy.")
		self.glyphs.glyphs.collect do |glyph, i|
			t = glyph.glyph
			#puts t.inspect
			#puts (t + 29).chr.inspect
			#get_font_mapping(t.chr)
			(t - 1).chr

#			(glyph.glyph + 29).chr
		end.join()
	end
end
class Pango::Rectangle
	def inspect()
		"#<Pango::Rectangle: #{x}, #{y}, #{width} x #{height}>"
	end
end
