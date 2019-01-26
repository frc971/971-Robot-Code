import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib
from gi.repository import Gdk
from gi.repository import GdkX11
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

    # Draw in response to an expose-event
    def __init__(self):
        super(BaseWindow, self).__init__()
        #self.window.connect("destroy", quit_main_loop)

        self.set_size_request(640, 600)
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
