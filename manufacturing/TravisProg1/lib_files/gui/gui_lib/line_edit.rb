require "gtk3"

require "gui/gtk_utils/colors.rb"
require "gui/gtk_utils/pango_inspect.rb"
require "gui/gtk_utils/clipping.rb"
require_relative "text_edit_event.rb"

class ExpLine
	def initialize(txt)
		@lay = get_desc_pango()
		@txt = txt
		update_txt()
	end
	def empty() return @txt.length == 0 end
	def draw(cr, cursor = nil, x = 0, y = 0)
		Color::Yellow.set(cr)
		cr.update_pango_layout(@lay)

		if (cursor)
			attrs = Pango::AttrList.new()
			attrs.insert(Color::Black.pattr_fg(cursor, cursor + 1))
			@lay.attributes = attrs
			p = @lay.index_to_pos(cursor)
			Color::Yellow.set(cr)
			ox, oy = x + p.x / Pango::SCALE, y + p.y / Pango::SCALE
			cr.rectangle(ox, oy, 6, 13)
			cr.fill()
		end

		cr.move_to(x, y)
		cr.show_pango_layout(@lay)
		@lay.attributes = nil if (cursor)
	end

	def height() return @lay.pixel_extents[1].height end

	def update_txt() @lay.set_text(@txt) end

	def key_press(ev, cursor)
		TextEditEvent.key_press(self, @txt, ev, cursor) 
	end
	def get_cursor_xpos(cursor)
		p = @lay.index_to_pos(cursor)
		return p.x / Pango::SCALE + (@xoff || 0)
	end
	def get_cursor_x(x)
		x -= (@xoff || 0)
		inside, index, trailing = @lay.xy_to_index(x * Pango::SCALE, 0)
		index += 1 if index + 1 <= @txt.length && x > 0 && !inside
		return index
	end
	def get_bbox(x, y)
		ext = @lay.pixel_extents[1]
		return SClipBBox.new(x + @xoff, y, ext.width + 6, ext.height)
	end
	attr_accessor :cursor, :xoff
end

class GenericFormattedLine
	def draw(cr, cursor = nil, x = 0, y = 0)
		Color::Yellow.set(cr)
		cr.update_pango_layout(@lay)

		if (cursor)
			cursor = cursor[0] if cursor.class != Integer
			attrs = Pango::AttrList.new()
			attrs.insert(Color::Black.pattr_fg(cursor, cursor + 1))
			@lay.attributes = attrs
			p = @lay.index_to_pos(cursor)
			Color::Yellow.set(cr)
			ox, oy = x + p.x / Pango::SCALE, y + p.y / Pango::SCALE
			cr.rectangle(ox, oy, 6, 13)
			cr.fill()
		end

		cr.move_to(x, y)
		cr.show_pango_layout(@lay)
		@lay.attributes = nil if (cursor)
	end
	def height() return @lay.pixel_extents[1].height end
	def key_press(ev, cursor)
		cur = cursor[0]
		g = 0
		items = get_items()
		items.each do |prefix, p, i|
			g += prefix.length
			cur = cursor[0] - g
			if cur <= p.length
				curref = [cur]
				if !TextEditEvent.key_press(self, p, ev, curref)
					k = ev.keyval
					if (k == Gdk::Keyval::KEY_Right && i < (items.length - 1))
						cursor[0] = g + p.length + items[i + 1][0].length
					elsif (k == Gdk::Keyval::KEY_Left && i > 0)
						cursor[0] = g - prefix.length
					else
						return false
					end
					return true
				end
				cursor[0] = g + curref[0]
				return true
			end
			g += p.length
		end
		return false
	end
	def get_cursor_xpos(cursor)
		p = @lay.index_to_pos(cursor)
		return p.x / Pango::SCALE + (@xoff || 0)
	end
	def cap_length(index, len)
		index = 0 if index < 0
		index = len if index >= len
		return index
	end
	def get_cursor_x(x)
		x -= (@xoff || 0)
		inside, index, trailing = @lay.xy_to_index(x * Pango::SCALE, 0)
		items = get_items()
		index += 1 if !inside
		g = 0
		items.each do |prefix, p, i|
			next_prefix = items.length - 1 > i ? items[i + 1][0] : nil

			g += prefix.length
			if (!next_prefix || index - g < p.length + 1 + next_prefix.length / 2)
				return g + cap_length(index - g, p.length)
			end
			g += p.length
		end
	end
	def get_bbox(x, y)
		ext = @lay.pixel_extents[1]
		return SClipBBox.new(x + @xoff, y, ext.width + 6, ext.height)
	end
	attr_accessor :cursor, :xoff
end

class ForStartLine < GenericFormattedLine
	A = "for ("
	B = "; "
	C = "; "
	D = "):"
	def initialize(a, b, c)
		@a = a
		@b = b
		@c = c
		@lay = get_desc_pango()
		update_txt()
	end
	def update_txt()
		@lay.set_text(A + @a + B + @b + C + @c + D)
	end
	def get_items()
		return [[A, @a, 0], [B, @b, 1], [C, @c, 2]]
	end
end

class IfStartLine < GenericFormattedLine
	A = "if ("
	D = "):"
	def initialize(a)
		@a = a
		@lay = get_desc_pango()
		update_txt()
	end
	def update_txt()
		@lay.set_text(A + @a + D)
	end
	def get_items()
		return [[A, @a, 0]]
	end
end

class ElseIfStartLine < GenericFormattedLine
	A = "else if ("
	D = "):"
	def initialize(a)
		@a = a
		@lay = get_desc_pango()
		update_txt()
	end
	def update_txt()
		@lay.set_text(A + @a + D)
	end
	def get_items()
		return [[A, @a, 0]]
	end
end
class ElseStartLine < GenericFormattedLine
	A = "else:"
	def initialize()
		@lay = get_desc_pango()
		update_txt()
	end
	def update_txt()
		@lay.set_text(A)
	end
	def key_press(ev, cursor) return false end
	def get_cursor_x(x) return A.length end
end

class FunctionStartLine < GenericFormattedLine
	A = "function "
	B = "("
	C = ") -> "
	D = ":"
	def initialize(fn)
		@name = fn.name
		@args = fn.args
		@ret_t = fn.ret_t
		@lay = get_desc_pango()
		update_txt()
	end
	def update_txt()
		@lay.set_text(A + @name + B + @args + C + @ret_t + D)
	end
	def get_items()
		return [[A, @name, 0], [B, @args, 1], [C, @ret_t, 2]]
	end
end

class ClassStartLine < GenericFormattedLine
	A = "class "
	B = ":"
	def initialize(cls)
		@name = cls.name
		@lay = get_desc_pango()
		update_txt()
	end
	def update_txt()
		@lay.set_text(A + @name + B)
	end
	def get_items()
		return [[A, @name, 0]]
	end
end

def compile_suite_to(suite, lines, xoff = 0)
	suite.each do |item|
		item.compile_to(lines, xoff)
	end
end



class ExpStmt
	def compile_to(lines, xoff)
		lines << exp = ExpLine.new(@exp)
		exp.xoff = xoff
	end
end

class IfStmt
	def compile_to(lines, xoff)
		i = 0
		while (i < @suites.length || i < @conds.length)
			suite = @suites[i] ||= []
			cond = @conds[i]
			if i == 0
				lines << if_stmt = IfStartLine.new(cond)
				if_stmt.xoff = xoff
			else
				if (cond)
					lines << if_stmt = ElseIfStartLine.new(cond)
					if_stmt.xoff = xoff
				else
					lines << if_stmt = ElseStartLine.new()
					if_stmt.xoff = xoff
				end
			end
			compile_suite_to(suite, lines, xoff + 10)
			i += 1
		end
	end
end

class ForStmt
	def compile_to(lines, xoff)
		lines << f_stmt = ForStartLine.new(@a, @b, @c)
		f_stmt.xoff = xoff
		compile_suite_to(@suite, lines, xoff + 10)
	end
end
