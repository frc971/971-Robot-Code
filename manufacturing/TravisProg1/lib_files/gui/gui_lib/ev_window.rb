require "gtk3"

require "gui/gtk_utils/pango_tools.rb"
require "gui/gtk_utils/colors.rb"
require "gui/gtk_utils/clipping.rb"

class EventWindow
	def initialize(wi = 640, hi = 480)
		w = Gtk::Window.new("Window name")
		w.set_size_request(wi, hi)
		w.move(800, 0)
		w.add(@d = Gtk::DrawingArea.new())
		w.signal_connect("delete_event") { Gtk.main_quit }
		@needs_draw = false
		# Keep the last 2 because of ref-counting issues.
		# I suspect that the update_pango_context which stores a
		# reference to the cairo_context in the pango context no longer
		# maintains that reference.
		# This is a problem currently for ruby version:
		# ruby 2.3.3p222 (2016-11-21 revision 56859) [x64-mingw32]
		# and gems:
		#  gtk3-3.1.0-x64-mingw32
		#  cairo-gobject-3.1.0-x64-mingw32
		#  cairo-1.15.4-x64-mingw32
		#  pango-3.1.0-x64-mingw32
		#  Sat 01/14/2017
		# Parker, Michael, & Travis
		@leak_cr = []
		@d.signal_connect("draw") { |w, cr| 
			# @leak_cr.push(cr)
			# @leak_cr.unshift if (@leak_cr.length() > 2)
			update_pango_context(cr)
			Color.new(0.1,0.1,0.1).set(cr)
			cr.paint()
			draw(cr)
#			obj.draw(cr)
			@needs_draw = true
		}
		GC.start()
		@d.set_events(Gdk::EventMask::ALL_EVENTS_MASK &
             		 ~Gdk::EventMask::SMOOTH_SCROLL_MASK)

		@d.set_can_focus(true)
		@d.set_can_default(true)
		@d.set_focus(true)

		[["button_press_event", :button_press_event], 
	 		["button_release_event", :button_release_event], 
			["motion_notify_event", :motion_notify_event],
			["key_press_event", :key_press],
			["key_release_event", :key_release],
			["scroll_event", :scroll_event],
			["configure_event", :configure_event],
		].each do |evname, evtype|
			if respond_to?(evtype)
				@d.signal_connect(evname) { |w, e|
					send(evtype, e)
				}
			elsif has_event_delegate(evtype)
				@d.signal_connect(evname) { |w, e|
					ev_delegate_send(evtype, e)
				}
			end
		end
		w.show_all()
	end
	def has_event_delegate(evtype) return false end
	def redraw()
		if (@needs_draw)
			@d.queue_draw()
			@needs_draw = false
		end
	end
	attr_accessor :widget
end

class EvWindow < EventWindow
	def initialize(obj, *args)
		@obj = obj
		super(*args)
	end
	def has_event_delegate(evtype)
		@obj.respond_to?(evtype)# || @obj.has_event_delegate(evtype)
	end
	def ev_delegate_send(evtype, ev)
		@obj.send(evtype, ev)
		redraw()
	end
	def configure_event(ev)
		changed = false
		if (@bbox == nil)
			changed = true
			@bbox = SClipBBox.new(0,0,ev.width, ev.height) 
		end
		if @bbox.w != ev.width
			@bbox.w = ev.width
			changed = true
		end
		if @bbox.h != ev.height
			@bbox.h = ev.height
			changed = true
		end
		@obj.resize(@bbox) if changed && @obj.respond_to?(:resize)
	end
	def draw(cr)
		@obj.draw(cr)
	end
	def self.run(w, *args)
		self.new(w, *args)
		Gtk.main()
	end
end
