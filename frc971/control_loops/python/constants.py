import argparse

arg_parser = argparse.ArgumentParser(description='spline_editor')
arg_parser.add_argument(
    'size',
    metavar='N',
    default=800,
    type=int,
    nargs='?',
    help="size of the screen")
args = arg_parser.parse_args()
SCREEN_SIZE = args.size

WIDTH_OF_ROBOT = 0.65
LENGTH_OF_ROBOT = 0.8

ROBOT_SIDE_TO_BALL_CENTER = 0.15 # Placeholder value
BALL_RADIUS = 0.165
ROBOT_SIDE_TO_HATCH_PANEL = 0.1 # Placeholder value
HATCH_PANEL_WIDTH = 0.4826

FIELD = 2020

if FIELD == 2019:
    WIDTH_OF_FIELD_IN_METERS = 8.258302 # Half Field
elif FIELD == 2020:
    WIDTH_OF_FIELD_IN_METERS = 15.98295 # Full Field
    LENGTH_OF_FIELD_IN_METERS = 8.21055 # Full Field

def pxToM(p, length = False):
    if(length):
        return p * LENGTH_OF_FIELD_IN_METERS / (SCREEN_SIZE/2)
    return p * WIDTH_OF_FIELD_IN_METERS / SCREEN_SIZE

def mToPx(m, length = False):
    if(length):
        return (m*(SCREEN_SIZE/2)/LENGTH_OF_FIELD_IN_METERS)
    return (m*SCREEN_SIZE/WIDTH_OF_FIELD_IN_METERS)

def inToM(i):
    return (i*0.0254)
