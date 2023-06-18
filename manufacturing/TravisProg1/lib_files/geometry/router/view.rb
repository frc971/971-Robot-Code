require "gui/gtk_utils/colors.rb"

require "gtk3"
require "pathname"

require "cairo"
require "gui/gui_lib/ev_window.rb"
require "gui/gtk_utils/pango_inspect.rb"
require "gui/gtk_utils/clipping.rb"
require "tokens/tokenizer_lib_base.rb"
require "gui/gui_lib/line_edit.rb"
require_relative "drag_window.rb"
require_relative "emit_gcode.rb"

def get_cursor_extents()
  lay = get_desc_pango()
  lay.set_text(" ")
  extents = lay.pixel_extents[1]
  return [extents.width, extents.height]
end

CursorExtents = get_cursor_extents()
class StringEditView
	attr_accessor :bbox
	def resize(bbox) @bbox = bbox end
	def xoff() @xoff || 2 end
	def set_xoff(xoff) @xoff = xoff; self end
	def initialize(txt)
		@lay = get_desc_pango()
		@txt = txt
		@cursor = 0
		update_txt()
	end
	def update_txt() @lay.set_text(@txt) end
	def draw(cr)
		y = 0
		x = xoff
		Color::Yellow.set(cr)
		cr.update_pango_layout(@lay)

		if (cursor)
			attrs = Pango::AttrList.new()
			attrs.insert(Color::Black.pattr_fg(cursor, cursor + 1))
			p = @lay.index_to_pos(cursor)
			Color::Yellow.set(cr)
			ox, oy = x + p.x / Pango::SCALE, y + p.y / Pango::SCALE
			if (@txt.length == cursor)
				cr.rectangle(ox, oy, CursorExtents[0], CursorExtents[1])
			else
				p2 = @lay.index_to_pos(cursor + 1)
				cr.rectangle(ox, oy, (p2.x - p.x) / Pango::SCALE, CursorExtents[1])
			end
			cr.fill()
			@lay.attributes = attrs
		end

		cr.move_to(x, y)
		cr.show_pango_layout(@lay)
		@lay.attributes = nil if (cursor)
		return y + height
	end
	def cursor() @cursor end
	def key_press(ev)
		cursor = [@cursor]
		TextEditEvent.key_press(self, @txt, ev, cursor)
		@cursor = cursor[0]
	end
	def button_press_event(ev, pt)
		@cursor = get_cursor_x(pt.x)
	end
	def get_cursor_x(x)
		x -= xoff
		inside, index, trailing = @lay.xy_to_index(x * Pango::SCALE, 0)
		index += 1 if index + 1 <= @txt.length && x > 0 && !inside
		return index
	end
	def take_focus(ev) self end
	def height() h = @lay.pixel_extents[1].height end
end

def rounded_rect(cr, x, y, w, h, r)
	cr.move_to(x + r, y)
  	cr.line_to(x, y + r) # should be arc
	cr.line_to(x, y + h - r)
  	cr.line_to(x + r, y + h) # should be arc
	cr.line_to(x + w - r, y + h)
		cr.line_to(x + w, y + h - r) # should be arc
	cr.line_to(x + w, y + r)
		cr.line_to(x + w - r, y) # should be arc
	cr.line_to(x + r, y)
end

class ActionButton
	def initialize(txt, obj, *action_args)
		@lay = get_desc_pango()
		@lay.set_text(txt)
		@obj = obj
		@action = action_args
	end
	Highlight = Color.new(0, 0, 0.75)
	Normal = Color.new(0.0, 0, 1.0)
	def draw(cr)
		w = 0
		(@hi ? Highlight : Normal).set(cr)
		rounded_rect(cr, 2, 2, width() - 4, height() - 4, 3)
		cr.fill()
		Color::Yellow.set(cr)
		cr.move_to(10, 5)
		cr.show_pango_layout(@lay)
	end
	def button_press_event(ev, pt)
		@obj.send(*@action)
	end
	def motion_notify_event(ev, pt) @hi = true end
	def exit_notify_event() @hi = false end
	def height() @lay.pixel_extents[1].height + 10 end
	def width() @lay.pixel_extents[1].width + 20 end
	attr_accessor :bbox
	def resize(bbox) @bbox = bbox end
end

class VerticalPacker
	class Padding
		def just_padding() true end
		def xoff() nil end
		def draw(cr) end
		def height() @height end
		def initialize(h) @height = h end
	end
	def initialize()
		@non_pad_children = []
		@children = []
	end
	def reset()
		@non_pad_children = []
		@children = []
	end
	def height()
		h = 0
		@children.each { |child| h += child.height }
		return h
	end
	def width()
		w = 0
		@children.each { |child| w = [w, child.width].max() }
		return w
	end
	attr_accessor :bbox
	def resize(bbox)
		@bbox = bbox
		w = @bbox.w
		y = 0 
		@children.each do |item|
			h = item.height
			if !(item.respond_to?(:just_padding) && item.just_padding())
				item.resize(SClipBBox.new(0, y, w, h))
			end
			y += h
		end
	end
	def children() @non_pad_children end
	def add(item)
		raise 'adding null item' if (!item)
		@children.push(item)
		if !(item.respond_to?(:just_padding) && item.just_padding())
			@non_pad_children.push(item)
		end
	end
	def draw(cr)
		cr.save()
		y = 0
		@children.each do |child|
			child.draw(cr)
			cr.translate(0, child.height)
		end
		cr.restore()
	end
end
class EmptyView
	def self.bbox() SClipBBox::InvalidBBox end 
	def self.resize(bbox) end
	def self.draw(cr)
	end
end
class SplitScreen
	attr_accessor :bbox
	def initialize(lhs, rhs, split)
		@children = [lhs, rhs]
		@split = split
	end
	def lhs() @children[0] end
	def rhs() @children[1] end
	def lhs=(child)
		@children[0] = child
		resize(@bbox) if @bbox
	end
	def rhs=(child)
		@children[0] = child
		resize(@bbox) if @bbox
	end
	def resize(bbox)
		@bbox = bbox
		a = bbox.dup()
		a.y = a.x = 0
		b = a.dup()
		b.x = @split
		a.w = @split
		b.w -= @split
		@children[0].resize(a)
		@children[1].resize(b)
	end
	def children()
		@children
	end
	def draw(cr)
		@children.each do |child|
			child.bbox.enter(cr) {
				child.draw(cr)
			}
		end
	end
end

class TextWindow
	attr_accessor :bbox
	def resize(bbox) @bbox = bbox end
	def initialize()
		@lay = get_desc_pango()
		@lay.set_text("hello world")
	end
	def draw(cr)
		cr.set_source_rgb(0.0,0.3,0.1)
		cr.paint()
		cr.set_source_rgb(1.0,1.0,0.0)
		cr.update_pango_layout(@lay)
		cr.move_to(2, 2)
		cr.show_pango_layout(@lay)
	end
end

class MyEventWrapper
  attr_accessor :ev, :x, :y
  def initialize(ev)
    @ev = ev
    @x = ev.x
    @y = ev.y
  end
end

class ClickTypeSplitView
	def initialize(tree)
		@tree = tree
	end
  def resize(bbox)
    @bbox = bbox
		@tree.resize(bbox.dup())
  end
	def event_recurse(ev, evnt_m, tree)
		x, y = ev.x, ev.y
		if tree.respond_to?(evnt_m)
			tree.send(evnt_m, ev.ev, Point2d.new(ev.x, ev.y))
			return tree, 0, 0
		end
		return nil, 0, 0 if !tree.respond_to?(:children)
		tree.children.each do |c|
			if (c.bbox.contain(x, y))
				ev.x -= c.bbox.x
				ev.y -= c.bbox.y
		#		puts "event_recurse: #{c.bbox.x}, #{c.bbox.y}"
				v, dx, dy = event_recurse(ev, evnt_m, c)
				return v, dx + c.bbox.x, dy + c.bbox.y if v
				ev.x,ev.y = x,y
			end
		end
		return nil, 0, 0
	end
	def scroll_event(ev)
		event_recurse(MyEventWrapper.new(ev), :scroll_event, @tree)
	end
	def button_press_event(ev)
		return if ev.time == @last_click_time
		@last_clicker, @lcx, @lcy = event_recurse(MyEventWrapper.new(ev), :button_press_event, @tree) 
		if @last_clicker && @last_clicker.respond_to?(:take_focus)
			@focus = @last_clicker.take_focus(ev)
		end
		@last_click_time = ev.time
	end
	def button_release_event(ev)
    pt = Point2d.new(ev.x, ev.y)
		if @last_clicker && @last_clicker.respond_to?(:button_release_event)
      pt = Point2d.new(ev.x, ev.y)
			pt.x -= @last_clicker.bbox.x
			pt.y -= @last_clicker.bbox.y
			@last_clicker.button_release_event(ev, pt)
		end
		@last_clicker = nil
	end
	def motion_notify_event(ev)
		if @last_clicker && @last_clicker.respond_to?(:motion_notify_event)
      pt = Point2d.new(ev.x, ev.y)
			pt.x -= @lcx
			pt.y -= @lcy
			@last_clicker.motion_notify_event(ev, pt)
		else
			obj, dx, dy = event_recurse(MyEventWrapper.new(ev), :motion_notify_event, @tree)
			if (@exit_notify && obj != @exit_notify)
				@exit_notify.exit_notify_event()
				@exit_notify = nil
			end
			@exit_notify = obj if (obj.respond_to?(:exit_notify_event))
		end
	end
  def key_press(ev)
		if @focus && @focus.respond_to?(:key_press)
			@focus.key_press(ev)
		end
  end
  def key_release(ev)
		if @focus && @focus.respond_to?(:key_release)
			@focus.key_release(ev)
		end
  end
	def draw(cr)
		@tree.draw(cr)
	end
end

class VerticalPackerPlan < Array
	def make(obj)
		pck = VerticalPacker.new()
		self.each do |item|
			if (item.is_a?(Symbol))
				pck.add(obj.send(item))
			else
				pck.add(item)
			end
		end
		return pck
	end
end

module DelegateToLayout
	def height() @layout.height end
	def width() @layout.width end
	def resize(bbox)
		@layout.resize(bbox)
	end
	def bbox() @layout ? @layout.bbox : nil end
	def children() @layout.children end
	def draw(cr) @layout.draw(cr) end
end

module Router
	class ConstStringLineView
		def initialize(str)
			@lay = get_desc_pango()
			@lay.set_text(str)
		end
		def draw_to(cr, x, y)
			#cr.set_source_rgb(1.0,1.0,0.0)
			cr.move_to(x, y)
			cr.update_pango_layout(@lay)
			cr.show_pango_layout(@lay)
			return y + height()
		end
		def height() @lay.pixel_extents[1].height end
	end
	class ConstStringView
		def initialize(str, col = nil)
			@col = col || Color::Yellow
			@lay = get_desc_pango()
			@lay.set_text(str)
		end
		def reset_text(txt) @lay.set_text(txt) end
		def just_padding() return true end
		attr_accessor :xoff
		def set_xoff(xoff) @xoff = xoff; self end
		def draw(cr)
			y = 0
			@col.set(cr)
			cr.move_to(@xoff || 0, y)
			cr.update_pango_layout(@lay)
			cr.show_pango_layout(@lay)
			return y + height()
		end
		def height() @lay.pixel_extents[1].height end
	end
	class ListView
		HoverC = Color::White
		DullC = Color::Yellow
		attr_accessor :bbox
		def resize(bbox) @bbox = bbox end
		def initialize(list, to_viewable, on_item_click)
			@on_item_click = on_item_click
			@views = list.collect(&to_viewable)
		end
		def draw_to(cr, x, y)
			@views.each.with_index do |item, i|
				(i == @hover_i ? HoverC : DullC).set(cr)
				y = item.draw_to(cr, x, y)
			end
			return y
		end
		def height() @height ||= @views.collect(&:height).inject(0, &:+) end
		def button_press_event(ev, cur)
			y = cur.y
			@views.each.with_index do |view, i|
				if (view.height() > y && y > 0)
					@on_item_click.call(i)
				end
				y -= view.height()
			end
		end
		def motion_notify_event(ev, cur)
			y = cur.y
			@views.each.with_index do |view, i|
				if (view.height() > y && y > 0)
					@hover_i = i
				end
				y -= view.height()
			end
		end
	end
	class SheetListView
		Sheets = ConstStringLineView.new("Select Sheet:")
		attr_accessor :bbox
		def resize(bbox)
			@bbox = bbox
      y = @scroll
			y += 2 + Sheets.height + 5
			@lv.bbox = SClipBBox.new(0, y, @bbox.w, h = @lv.height())
			y += h
		end
		def children() [@lv] end
		def initialize(sheets, master)
			@sheets = sheets
			@lv = ListView.new(@sheets, lambda {|a| ConstStringLineView.new("sheet #{a.sheet_name}") },lambda { |i|
				master.set_sheet_view(master.open_sheet(i))
			})
      @scroll = 0
		end
    def scroll_event(ev, pt)
      return if !ev.state.shift_mask?
		  if (ev.direction == Gdk::ScrollDirection::DOWN)
			  @scroll -= (Sheets.height * 3).to_i
	   	elsif (ev.direction == Gdk::ScrollDirection::UP)
			  @scroll += (Sheets.height * 3).to_i
		  end
      resize(@bbox)
    end
		def draw(cr)
			x, y = 2, 2 + @scroll
			Color::Yellow.set(cr)
			y = Sheets.draw_to(cr, x, y) 
			@lv.draw_to(cr, x + 5, y + 5)
		end
	end
	class RouterLayoutView
		include DelegateToLayout
		def initialize(parts_lib, sheets)
			@parts_lib = parts_lib
			@sheets = sheets
			@pm_loader = PMLoader.new()
			@pm_loader.parts_library = @parts_lib
			@layout = SplitScreen.new(EmptyView, @drag_window = DragWindow.new(@pm_loader), 400)
			set_sheet_list_view()

#			set_sheet_view(sheet = open_sheet(0))
#			if sheet.cuts.length == 0
#				@layout.lhs.cut_name = "cut1"
#				@layout.lhs.add_cut()
#			else
#				sheet.set_active(sheet.cuts[0])
#			end
		end
		def open_sheet(i)
			@sheets.open_sheet(i, @pm_loader)
		end
		def set_sheet_list_view()
			@layout.lhs = SheetListView.new(@sheets, self)
		end
		def set_sheet_view(sheet)
			SheetView.make(sheet, @parts_lib, self).config_main_view(self)
		end
		def drag_window() @drag_window end
		def set_active_view(view)
			@layout.lhs = view
		end
	end
	class SheetView
		include DelegateToLayout
		def initialize(sheet, parts_lib, master_view)
			@master_view = master_view
			@sheet, @parts_lib = sheet, parts_lib
			@layout = LayoutPlan.make(self)
			@add_entity_view = AddEntityView.new(self, @parts_lib)
			@add_hole_view = AddHoleView.new(self)
		end
		def config_main_view(main_view)
			if @sheet.empty?()
				main_view.set_active_view(NewSheetCutView.new(self))
			else
				main_view.set_active_view(self)
			end
			main_view.drag_window.set_sheet(@sheet)
		end
		def self.make(sheet, parts_lib, master_view)
			sheet_view = self.new(sheet, parts_lib, master_view)
		end
		def add_cut(cut_name)
			set_active_cut(@sheet.add_cut(cut_name))
			@active_cut_selector.set_current_cut()
			@active_cut_selector.update_cut_list()
		end
		def set_active_cut(cut)
			@sheet.set_active(cut)
			@sheet.sheet_info.set_active(cut)
			@master_view.set_active_view(self)
		end
		def enter_add_cut_view()
			@master_view.set_active_view(NewSheetCutView.new(self))
		end
		def add_sheet_cmds()
			return (V2HLayout
				.add(ActionButton.new("Add Hole", self, :add_hole))
				.add(ActionButton.new("Add Item", self, :add_entity))
				.add(ActionButton.new("Del Select", self, :del_select))
			)
		end
		def add_rescan_cmds()
			return (V2HLayout
				.add(ActionButton.new("Rescan parts_library", self, :rescan))
			)
		end
		def rescan()
			@parts_lib.rescan()
			@add_entity_view = AddEntityView.new(self, @parts_lib)
		end
		def del_select()
			active_cut = @sheet.active_cut
			return puts "no active cut." if (!active_cut)
			layout = active_cut.layout
			select = layout.selected
			return puts "nothing selected." if (!select)
			layout.delete(select)
			layout.selected = nil
		end
		def select_active_cut()
			@active_cut_selector ||= ActiveCutSelector.new(self, @sheet)
		end
		def add_cut_cmds()
			return (V2HLayout
				.add(ActionButton.new("Emit Gcode", self, :emit_gcode))
				.add(ActionButton.new("Save", self, :save_all_changes))
				.add(ActionButton.new("Add Cut", self, :enter_add_cut_view))
			)
		end
		def emit_gcode()
			if (@sheet.active_cut == nil)
				puts "first select active cut before emitting gcode."
				return
			end
			@master_view.set_active_view(EmitGcodeView.new(@sheet, self, @master_view))
		end
		def save_all_changes()
			@sheet.save_changes()
		end
		def add_hole()
			@master_view.set_active_view(@add_hole_view)
			@master_view.drag_window.set_active_adder(@add_hole_view)
		end
		def add_entity()
			@master_view.set_active_view(@add_entity_view)
			@master_view.drag_window.set_active_adder(@add_entity_view)
		end
		def done_adding_items()
			@master_view.set_active_view(self)
			@master_view.drag_window.set_active_adder(nil)
		end
		def show_sheet_name()
			ConstStringView.new("Editing Sheet: #{@sheet.sheet_name}").set_xoff(2)
		end

		LayoutPlan = VerticalPackerPlan.new([
			VerticalPacker::Padding.new(2),
			:show_sheet_name,
			:add_sheet_cmds,
			:add_rescan_cmds,
#			VerticalPacker::Padding.new(5),
			VerticalPacker::Padding.new(5),
			:add_cut_cmds,
			:select_active_cut,
		])
	end
	class NewSheetCutView
		include DelegateToLayout
		attr_accessor :cut_name
		def initialize(sheet_view)
			@sheet_view = sheet_view
			@cut_name = ""
			@layout = LayoutPlan.make(self)
		end
		def add_cut()
			if @cut_name.length == 0
				puts "error! cut filename is empty"
				return
			end
			@sheet_view.add_cut(@cut_name)
		end
		def sheet_name_edit()
			return StringEditView.new(@cut_name)
		end
		def add_cut_button()
			return V2HLayout.add(ActionButton.new("Add Sheet Cut", self, :add_cut))
		end
		LayoutPlan = VerticalPackerPlan.new([
			VerticalPacker::Padding.new(2),
			ConstStringView.new("Adding new sheet cut. Enter filename:").set_xoff(2),
			:sheet_name_edit,
			:add_cut_button,
		])
	end
	class AddEntityView
		include DelegateToLayout
		def initialize(sheet_view, parts_lib)
			@sheet_view, @parts_lib = sheet_view, parts_lib
			@part_selector = PartSelectorView.new(@parts_lib, self)
			@layout = LayoutPlan.make(self)
		end
		def part_selector()
			@part_selector
		end
		def add_items() @sheet_view.done_adding_items() end
		def to_sheet_view()
			return V2HLayout.add(ActionButton.new("Done Adding Items", self, :add_items))
		end
		def current_item
			@part_selector.current_item()
		end
		LayoutPlan = VerticalPackerPlan.new([
			:to_sheet_view,
			VerticalPacker::Padding.new(2),
			ConstStringView.new("Adding new part by clicking\n while holding a:").set_xoff(2),
			:part_selector,
			#:sheet_name_edit,
			#:add_cut_button,
		])
	end
	class AddHoleView
		include DelegateToLayout
		def initialize(sheet_view)
			@sheet_view = sheet_view
			@layout = LayoutPlan.make(self)
		end
		def current_item()
			HoleSpec
		end
		def done_adding_holes() @sheet_view.done_adding_items() end
		def to_sheet_view()
			return V2HLayout.add(ActionButton.new("Done Adding Holes", self, :done_adding_holes))
		end
		LayoutPlan = VerticalPackerPlan.new([
			:to_sheet_view,
			VerticalPacker::Padding.new(2),
			ConstStringView.new("Adding new hole by clicking\n while holding a.").set_xoff(2),
			#:sheet_name_edit,
			#:add_cut_button,
		])
	end
	class EmitGcodeView
		include DelegateToLayout
		def initialize(sheet, sheet_view, master_view)
			@sheet = sheet
			@sheet_view = sheet_view
			@master_view = master_view
			cut_fname = @sheet.active_cut.fname.basename
			cut_name = cut_fname.to_path[0...-(cut_fname.extname.length)]
			@fname = "output/#{Time.new().strftime("%Y%m%d/#{cut_name}.ngc")}"
			Pathname.new(@fname).dirname.mkpath
			@gcode_groups = @sheet.active_cut.make_gcode_groupings()
			#@origin_status = ConstStringView.new("origin currently unset.").set_xoff(2),
			@layout = LayoutPlan.make(self)
			@origin_picker = OriginSelector.new(@sheet)
			@master_view.drag_window.update_origin_picker(@origin_picker)
		end
		def done_adding_holes()
			@sheet_view.done_adding_items()
			@master_view.drag_window().update_sheet_layout(@sheet.active_cut.layout)
		end
		def to_sheet_view()
			return V2HLayout.add(ActionButton.new("Done Emitting Gcode", self, :done_adding_holes))
		end
		def select_origin()
			return V2HLayout.add(ActionButton.new("Set Origin", self, :do_select_origin))
		end
		def do_select_origin()
			if (!@origin_picker.selected)
				puts "could not set origin"
				return
			end
			# TODO(parker): this might just need to be deleted.
		end
		def emit_group(group)
			origin = @origin_picker.selected
			return puts "origin not set!" if (!origin)
			@gcode_groups.emit_group(group, @fname, origin)
		end
		def emit_holes()
			origin = @origin_picker.selected
			return puts "origin not set!" if (!origin)
			@gcode_groups.emit_holes(@fname, @sheet.sheet_info, origin) 
		end
		def list_gcode_groups()
			@list_gcode_groups ||= (
				vp = VerticalPacker.new();
				vp.add(V2HLayout.add(ActionButton.new("Emit holes", self, :emit_holes))) if (@gcode_groups.has_holes());
				@gcode_groups.groups.each do |name, group|
					vp.add(V2HLayout.add(ActionButton.new("Emit #{name.inspect}", self, :emit_group, group)));
				end
				vp;
			)
			return @list_gcode_groups
		end
		def gcode_name_edit()
			return StringEditView.new(@fname)
		end
		LayoutPlan = VerticalPackerPlan.new([
			:to_sheet_view,
			VerticalPacker::Padding.new(2),
			ConstStringView.new("Set Gcode filename:").set_xoff(2),
			:gcode_name_edit,
			#:select_origin,
			ConstStringView.new("Select Gcode grouping to emit.").set_xoff(2),
			:list_gcode_groups
			#:sheet_name_edit,
			#:add_cut_button,
		])
	end
	class ActiveCutSelector
		include DelegateToLayout
		def initialize(sheet_view, sheet)
			@sheet_view = sheet_view
			@sheet = sheet
			@current_cut = ConstStringView.new(" ").set_xoff(2)
			@cut_list = VerticalPacker.new()
			set_current_cut()
			update_cut_list()
			@layout = LayoutPlan.make(self)
		end
		def set_current_cut()
			if (cut = @sheet.active_cut)
				@current_cut.reset_text("current cut: #{cut.fname.basename.to_path}")
			else
				@current_cut.reset_text("no active cut. Select One.")
			end
		end
		def select_cut(cut)
			@sheet_view.set_active_cut(cut)
			set_current_cut()
		end
		def update_cut_list()
			@cut_list.reset()
			@sheet.cuts.each do |cut|
				@cut_list.add(V2HLayout.add(ActionButton.new(
					"cut: #{cut.fname.basename.to_path}", self, :select_cut, cut)))
			end
			@sheet_view.resize(@sheet_view.bbox) if @sheet_view.bbox
		end
		attr_accessor :current_cut, :cut_list
		LayoutPlan = VerticalPackerPlan.new([
			:current_cut,
			#:cut_list,
		])
	end
	class PartSelectorView
		include DelegateToLayout
		def initialize(parts_lib, to_update)
			@to_update = to_update
			@parts_lib = parts_lib
			@parts_lib_stack = [@parts_lib]
			make_layout()
		end
		def current_item()
			spec = @parts_lib.spec
			return (!spec || spec.is_error) ? nil : spec
		end
		def up_dir()
			@parts_lib_stack.pop()
			@parts_lib = @parts_lib_stack[-1]
			update_layout()
		end
		def down_dir(sub_lib)
			@parts_lib = sub_lib
			@parts_lib_stack.push(sub_lib)
			update_layout()
		end
		def update_layout()
			make_layout()
			@to_update.resize(@to_update.bbox) if @to_update.bbox
		end
		def make_layout()
			packer = VerticalPacker.new()
			if (@parts_lib_stack.length > 1)
				packer.add(V2HLayout.add(ActionButton.new("..", self, :up_dir)))
			end
			@parts_lib.sub_libs.each do |sub_lib|
				packer.add(V2HLayout.add(ActionButton.new(sub_lib.fname(), self, :down_dir, sub_lib)))
			end
			if spec = @parts_lib.spec()
				if (spec.is_error)
					packer.add(ConstStringView.new("error: #{spec.msg}").set_xoff(2))
				else
					packer.add(ConstStringView.new("cur_part: #{spec.name}").set_xoff(2))
				end
			elsif (@parts_lib_stack.length > 1)
				packer.add(ConstStringView.new("current folder: #{@parts_lib.fname()}").set_xoff(2))
			end
			@layout = packer
		end
		#def resize(bbox) @bbox = bbox end
	end
end
class V2HLayout
	def self.add(item) self.new().add(item) end
	def initialize()
		@items = []
		@height = 0
	end
	def add(item)
		@items << item
		@height = [@height, item.height].max()
		self
	end
	def children() @items end
	def height() @height end
	def resize(bbox)
		@bbox = bbox
		x = 0
		@items.each do |item|
			w = item.width
			item.resize(SClipBBox.new(x, 0, w, @height))
			x += w
		end
	end
	attr_accessor :bbox
	def draw(cr)
		@items.each do |item|
			item.bbox.enter(cr) { item.draw(cr) }
		end
	end
end
