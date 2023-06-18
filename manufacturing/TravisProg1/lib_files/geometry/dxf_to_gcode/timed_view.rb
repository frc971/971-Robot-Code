require "gtk3"

require "cairo"
require "gui/gui_lib/ev_window.rb"
require "gui/gtk_utils/pango_inspect.rb"
require "gui/gtk_utils/clipping.rb"
require "tokens/tokenizer_lib_base.rb"

class TimeSeriesViewer
	class Point2d
		init_fields :x, :y
		def -(o) Point2d.new(@x - o.x, @y - o.y) end
		def +(o) Point2d.new(@x + o.x, @y + o.y) end
	end
	def set_scale(scale) @scale = scale end
	def initialize()
		@scale = 1.0
		@xoff = 200
		@yoff = 200
	end
	def draw_fn(&blk)
		@draw_fn = blk
	end
  def resize(bbox)
    @bbox = bbox
  end
	def scroll_event(ev)
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
	def click_fn(&blk) @click_fn = blk end
	def drag_fn(&blk) @drag_fn = blk end
	def key_press_fn(&blk) @key_press_fn = blk end 
	def key_release_fn(&blk) @key_release_fn = blk end 
	def to_model(ev)
		xcur = (ev.x - @xoff) / @scale
		ycur = (@yoff - ev.y) / @scale
		return Point2d.new(xcur, ycur)
	end
	def button_press_event(ev)
		if (ev.button == 1)
			@click_point = Point2d.new(ev.x, ev.y)
			@click_fn.call(ev, to_model(ev)) if @click_fn
		end
	end
	def button_release_event(ev)
		@click_point = nil if (ev.button == 1)
	end
	def motion_notify_event(ev)
		if @click_point
			if !(@drag_fn && @drag_fn.call(to_model(ev)))
				cp = Point2d.new(ev.x, ev.y)
				delta = cp - @click_point
				@click_point = cp
				@xoff += delta.x
				@yoff += delta.y
			end
		end
	end
  def key_press(ev)
		@key_press_fn.call(ev) if @key_press_fn
  end
  def key_release(ev)
		@key_release_fn.call(ev) if @key_release_fn
  end
	def draw(cr)
		cr.set_source_rgb(0.0,0.1,0.1)
		cr.paint()
		cr.translate(@xoff, @yoff)
		cr.set_source_rgb(0.0,1.0,0.1)
		@draw_fn.call(cr, @scale) if (@draw_fn)			
	end
end
