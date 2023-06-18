require "geometry/placement/sheet.rb"
require "geometry/placement/drag_drop_rot_handler.rb"
require "geometry/placement/click_tester.rb"

module Router
	class HoleSpec
		def self.direct_load()
			r = HoleSpec::Radius
			pts = 16.times.collect do |i|
				th = i * Math::PI * 2 / 16.0
				Point2d.new(Math.cos(th) * r, Math.sin(th) * r)
			end
			pm = ProfileCrossing.new()
			pm.filename = self
			16.times do |i|
				pm.emit(pts[i], pts[i - 1])
			end
			return pm
		end
	end
	class GcodeSpec
		def direct_load()
			part_spec = self
			fname = part_spec.dxf_name()
			items = get_entities(fname)
			items2d = items.collect { |item| item.to_2d() }

			items2d_orient = Oriented.import_2d(items2d)
			items = [TwinableProfile.new(items2d_orient)]
			#items2d_orient = SweepLine.stitch_curves(items2d_orient)
#			items = items2d_orient

			scale = IntegerScale
			swl_p = offset_dxf_integer(items, scale, 2**16, 0.25)
#			scale_points(swl_p, 1, 1)
			pm = ProfileCrossing.new()
			pm.filename = part_spec
			swl_p.to_crossing(scale, pm)
			for item in items
				pm.add_only_drawable(item)
			end
			return pm
		end
	end
	class SheetCut
		def add_originable_points(lst)
			@layout.items.each do |item|
				lst << item.c if (item.item.filename == HoleSpec)
			end
		end
	end
	class SheetInfo
		def add_originable_points(lst)
			lst << Point2d.new(0, 0)
			lst << Point2d.new(@w, 0)
			lst << Point2d.new(0, @h)
			lst << Point2d.new(@w, @h)
		end
	end
	class MetalSheet
		def originable_points()
			lst = []
			@cuts.each do |cut|
				cut.add_originable_points(lst)
			end
			@sheet_info.add_originable_points(lst)
			return lst
		end
	end
end

class OriginSelector
	def initialize(sheet)
		@points = sheet.originable_points()
		@selected = nil
	end
	attr_accessor :selected
	def pick(q)
		@points.each do |point|
			dist = (point - q)
			dist = dist.x * dist.x + dist.y * dist.y
			if dist < 4 * Router::HoleSpec::Radius ** 2
				@selected = point
			end
		end
		return nil
	end
	def draw_scaled(cr, scale)
		if (@selected)
			cr.set_source_rgb(1.0, 0.5, 0.5)
			draw_crosshair(cr, scale, @selected)
		end
	end
end

class PMLoader
	def initialize()
		@cache = {}
	end
	def load_pm(part_spec)
		return nil if (part_spec == nil)
		@cache[part_spec] ||= part_spec.direct_load()
	end
	def load_pm_string(fname)
		puts "loading part: #{fname}"
		load_pm(@parts_library.realize(fname))
	end
	attr_accessor :parts_library
end

class DragWindow
	attr_accessor :bbox
	def resize(bbox)
		@bbox = bbox
	end
	def add_item(layout, cp)
		if (@adder && c_item = @adder.current_item())
			layout.add_item(@pm_loader.load_pm(c_item), cp, 0)
#				@pm_loader.load_pm("/home/parker/repo/geometry/gcode/sample/gcode.dxf"), cp, 0)
		end
	end
	def set_active_adder(adder)
		@adder = adder
	end
	def set_sheet(sheet)
		@sheet = sheet
		@sheet.active_cut_change() { |cut|
			update_sheet_layout(cut.layout)
		}
	end
	def update_origin_picker(origin_picker)
		if (@drag_drop_handler)
			@drag_drop_handler.layout.selected = nil
		end
		@origin_picker = origin_picker 
		ts = self
		@drag_drop_handler = nil
		ts.click_fn() do |ev, cp|
			@origin_picker.pick(cp)
		end
		ts.drag_fn() do |cp|
		end
		ts.key_press_fn() do |kv|
		end
		ts.key_release_fn() do |kv|
		end
		ts.draw_fn() do |cr, scale|
			@origin_picker.draw_scaled(cr, scale)
			GC.start()
		end
	end
	def update_sheet_layout(layout)
		ts = self
		drag_drop_handler = DragDropRotHandler.new(layout)
		@drag_drop_handler = drag_drop_handler
		drag_drop_handler.add_item_fn() do |cp|
			add_item(layout, cp)
		end
		ts.click_fn() do |ev, cp|
			drag_drop_handler.click(ev, cp)
		end
		ts.drag_fn() do |cp|
			drag_drop_handler.drag(cp)
		end
		ts.key_press_fn() do |kv|
			drag_drop_handler.key_press_fn(kv)
		end
		ts.key_release_fn() do |kv|
			drag_drop_handler.key_release_fn(kv)
		end
		ts.draw_fn() do |cr, scale|
			drag_drop_handler.draw_scaled(cr, scale)
			GC.start()
		end
	end
	def set_scale(scale) @scale = scale end
	def initialize(pm_loader)
		@pm_loader = pm_loader
		@scale = 20.0
		@xoff = 200
		@yoff = 200
	end
	def take_focus(ev) self end
	def draw_fn(&blk)
		@draw_fn = blk
	end
	def scroll_event(ev, cur)
#		puts "#{ev.x} #{ev.y}"
		#ev.x = @xoff + xcur * @scale
		#ev.y = @yoff - ycur * @scale
		xcur = (ev.x - @xoff) / @scale
		ycur = (@yoff - ev.y) / @scale
		if (ev.direction == Gdk::ScrollDirection::DOWN)
			@scale *= 1.1
		elsif (ev.direction == Gdk::ScrollDirection::UP)
			@scale /= 1.1
		end
		@xoff = ev.x - xcur * @scale
		@yoff = ev.y + ycur * @scale
	end
  def key_press(ev)
		@key_press_fn.call(ev) if @key_press_fn
  end
  def key_release(ev)
		@key_release_fn.call(ev) if @key_release_fn
  end
	def click_fn(&blk) @click_fn = blk end
	def drag_fn(&blk) @drag_fn = blk end
	def key_press_fn(&blk) @key_press_fn = blk end 
	def key_release_fn(&blk) @key_release_fn = blk end 
	def to_model(ev)
		xcur = (ev.x - @xoff) / @scale
		ycur = (@yoff - ev.y) / @scale
		return Point2d.new(xcur, ycur)
	end
	def button_press_event(ev, cp)
		if (ev.button == 1)
			@click_point = Point2d.new(cp.x, cp.y)
			@click_fn.call(ev, to_model(cp)) if @click_fn
		end
	end
	def button_release_event(ev, pt)
		@click_point = nil if (ev.button == 1)
	end
	def motion_notify_event(ev, cp)
		if @click_point
			if !(@drag_fn && @drag_fn.call(to_model(cp)))
				delta = cp - @click_point
				@click_point = cp
				@xoff += delta.x
				@yoff += delta.y
			end
		end
	end
	def draw(cr)
		if (@sheet)
			cr.set_source_rgb(0.0,0.1,0.1)
			cr.paint()
			cr.translate(@xoff, @yoff)
			cr.set_source_rgb(0.0,1.0,0.1)
			@sheet.draw_others(cr, @scale, @drag_drop_handler ? @drag_drop_handler.layout : nil)
			@draw_fn.call(cr, @scale) if (@draw_fn)			
		end
	end
end
module Router
	class MetalSheet
		def draw_others(cr, scale, layout)
			@sheet_info.draw_bounds(cr, scale)
			@cuts.each do |cut|
				cut.layout.draw_to(cr, scale, cut != @active_cut) if (cut.layout != layout)
			end
		end
	end
	class SheetInfo
		def draw_bounds(cr, scale)
			cr.set_source_rgb(1.0, 0.5, 0.0)
			bounds = sheet_dxf()
			if bounds
				bounds.each do |bound|
					bound.draw_to(cr, scale) #.each { |sitem| sitem.draw_to(cr, scale) }
				end
				return
			end
			w, h = @w * scale, @h * scale
			cr.move_to(0, 0)
			cr.line_to(w, 0)
			cr.line_to(w, -h)
			cr.line_to(0, -h)
			cr.line_to(0, 0)
			cr.stroke()
		end
	end
end

