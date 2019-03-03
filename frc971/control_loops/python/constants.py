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

WIDTH_OF_FIELD_IN_METERS = 8.258302

WIDTH_OF_ROBOT = 0.65
LENGTH_OF_ROBOT = 0.8

ROBOT_SIDE_TO_BALL_CENTER = 0.15 #Placeholder value
BALL_RADIUS = 0.165
ROBOT_SIDE_TO_HATCH_PANEL = 0.1 #Placeholder value
HATCH_PANEL_WIDTH = 0.4826

def pxToM(p):
    return p * WIDTH_OF_FIELD_IN_METERS / SCREEN_SIZE

def mToPx(m):
    return (m*SCREEN_SIZE/WIDTH_OF_FIELD_IN_METERS)
