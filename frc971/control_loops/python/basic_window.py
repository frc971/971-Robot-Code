import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib
from gi.repository import Gdk
from gi.repository import GdkX11
from color import Color, palette
import cairo

identity = cairo.Matrix()


# Override the matrix of a cairo context.
class OverrideMatrix(object):
    def __init__(self, cr, matrix):
        self.cr = cr
        self.matrix = matrix

    def __enter__(self):
        self.cr.save()
        self.cr.set_matrix(self.matrix)

    def __exit__(self, type, value, traceback):
        self.cr.restore()


mainloop = GLib.MainLoop()

def set_color(cr, color):
    if color.a == 1.0:
        cr.set_source_rgb(color.r, color.g, color.b)
    else:
        cr.set_source_rgba(color.r, color.g, color.b, color.a)

def quit_main_loop(*args):
    mainloop.quit()


def RunApp():
    try:
        mainloop.run()
    except KeyboardInterrupt:
        print('\nCtrl+C hit, quitting')
        mainloop.quit()


# Create a GTK+ widget on which we will draw using Cairo
class BaseWindow(Gtk.DrawingArea):
    def method_connect(self, event, cb):
        def handler(obj, *args):
            cb(*args)

        self.window.connect(event, handler)

    # Draw in response to an expose-event
    def __init__(self):
        super(BaseWindow, self).__init__()
        self.window = Gtk.Window()
        self.window.set_title("DrawingArea")
        self.window.connect("destroy", quit_main_loop)
        self.window.set_events(Gdk.EventMask.BUTTON_PRESS_MASK
                               | Gdk.EventMask.BUTTON_RELEASE_MASK
                               | Gdk.EventMask.POINTER_MOTION_MASK
                               | Gdk.EventMask.SCROLL_MASK
                               | Gdk.EventMask.KEY_PRESS_MASK)
        self.method_connect("key-press-event", self.do_key_press)
        self.method_connect("button-press-event",
                            self._do_button_press_internal)
        self.method_connect("configure-event", self._do_configure)

        self.set_size_request(640, 400)
        self.window.add(self)
        self.window.show_all()
        self.center = (0, 0)
        self.shape = (640, 400)
        self.needs_redraw = False

    def get_current_scale(self):
        w_w, w_h = self.window_shape
        w, h = self.shape
        return min((w_w / w), (w_h / h))

    def init_extents(self, center, shape):
        self.center = center
        self.shape = shape

    # The gtk system creates cr which is a cairo_context_t (in the c docs), and then it
    # passes it as a function argument to the "draw" event.  do_draw is the default name.
    def do_draw(self, cr):
        cr.save()
        cr.set_font_size(20)
        cr.translate(self.window_shape[0] / 2, self.window_shape[1] / 2)
        scale = self.get_current_scale()
        cr.scale(scale, -scale)
        cr.translate(-self.center[0], -self.center[1])
        self.needs_redraw = False
        self.handle_draw(cr)
        cr.restore()

    # Handle the expose-event by drawing
    def handle_draw(self, cr):
        pass

    def do_key_press(self, event):
        pass

    def _do_button_press_internal(self, event):
        o_x = event.x
        o_y = event.y
        x = event.x - self.window_shape[0] / 2
        y = self.window_shape[1] / 2 - event.y
        scale = self.get_current_scale()
        event.x = x / scale + self.center[0]
        event.y = y / scale + self.center[1]
        self.do_button_press(event)
        event.x = o_x
        event.y = o_y

    def do_button_press(self, event):
        pass

    def _do_configure(self, event):
        self.window_shape = (event.width, event.height)

    def redraw(self):
        if not self.needs_redraw:
            self.needs_redraw = True
            self.window.queue_draw()
