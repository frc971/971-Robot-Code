class DragDropRotHandler
	def initialize(layout)
		@layout = layout
		@picked = nil
		@click_point = nil
		@rot_point = nil
		@ctrl_held = false
		@a_held = false
	end
	def add_item_fn(&blk) @add_item_fn = blk end
	def click(ev, cp)
		if @a_held
			@add_item_fn.call(cp) if @add_item_fn
			@picked = nil
			return
		end
		if @ctrl_held
			@click_point = nil
			if (!@rot_point)
				@picked = @layout.pick(cp)
				@rot_point = @picked ? cp : nil
			else
				@rot_drag_point = cp
			end
		else
			@picked = @layout.pick(cp)
			@click_point = cp
		end
	end
	def drag(cp)
		if @picked
			if @click_point
				delta = (cp - @click_point)
				@picked.update(delta)
				@click_point = cp
			elsif @rot_drag_point
				a1 = @rot_drag_point - @rot_point
				a2 = cp - @rot_point
				dang = Math.atan2(a2.y, a2.x) - Math.atan2(a1.y, a1.x)
				@picked.rotate(@rot_point, dang)
				@rot_drag_point = cp
			end
		end
		return @picked ? true : nil
	end
	def key_release_fn(kv)
		if (Gdk::Keyval::KEY_Control_L == kv.keyval || Gdk::Keyval::KEY_Control_R == kv.keyval)
			@rot_point = nil
			@rot_drag_point = nil
			@ctrl_held = false
		end
		if (Gdk::Keyval::KEY_a == kv.keyval)
			@a_held = false
		end
	end
	def key_press_fn(kv)
		if (Gdk::Keyval::KEY_Control_L == kv.keyval || Gdk::Keyval::KEY_Control_R == kv.keyval)
			@ctrl_held = true
		end
		if (Gdk::Keyval::KEY_a == kv.keyval)
			@a_held = true
		end
	end
	def draw_scaled(cr, scale)
		if @rot_point
			cr.set_source_rgb(1.0, 0.5, 0.0)
			draw_crosshair(cr, scale, @rot_point)
		end
		@layout.draw_to(cr, scale)
	end
	attr_accessor :layout
end
