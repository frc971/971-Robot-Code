from gi.repository import Gtk

window = Gtk.Window()
screen = window.get_screen()

#Set screen size for rest of program.
SCREEN_SIZE = screen.get_height() / 3

WIDTH_OF_ROBOT = 0.65
LENGTH_OF_ROBOT = 0.8

# Placeholder value
ROBOT_SIDE_TO_BALL_CENTER = 0.15
BALL_RADIUS = 0.165

# Placeholder value
ROBOT_SIDE_TO_HATCH_PANEL = 0.1
HATCH_PANEL_WIDTH = 0.4826

FIELD = 2020

if FIELD == 2019:
    # Half Field
    WIDTH_OF_FIELD_IN_METERS = 8.258302
elif FIELD == 2020:
    # Full Field
    WIDTH_OF_FIELD_IN_METERS = 15.98295
    LENGTH_OF_FIELD_IN_METERS = 8.21055


def pxToM(p):
    return p * WIDTH_OF_FIELD_IN_METERS / SCREEN_SIZE


def mToPx(m):
    return (m * SCREEN_SIZE / WIDTH_OF_FIELD_IN_METERS)


def inToM(i):
    return (i * 0.0254)
