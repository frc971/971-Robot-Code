import cv2 as cv
import enum
import numpy as np


class Rect:
    """
    Holds points for a rectangle in an image.
    This section of the image is where to expect a ball.
    """

    # x1 and y1 are top left corner, x2 and y2 are bottom right
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return "({}, {}), ({}, {})".format(self.x1, self.y1, self.x2, self.y2)


class Alliance(enum.Enum):
    RED = enum.auto()
    BLUE = enum.auto()
    UNKNOWN = enum.auto()


class Letter(enum.Enum):
    A = enum.auto()
    B = enum.auto()


class Path:
    """
    Each path (ex. Red A, Blue B, etc.) contains a Letter, Alliance, and
    2-3 rectangles (the places to expect balls in).
    There may be only 2 rectangles if there isn't a clear view at all of the balls.
    """

    def __init__(self, letter, alliance, rects):
        self.letter = letter
        self.alliance = alliance
        self.rects = rects

    def __str__(self):
        return "%s %s: " % (self.alliance.value, self.letter.value)


# TODO: view each of the 4 images in this folder by running `./img_viewer.py <image_file>`,
# and figure out the retangle bounds for each of the 3 balls in each of the 4 paths.
# You can move your cursor to the endpoints of the rectangle, and it will show
# the coordinates.
# Note that in some images, there might not be a good view of 3 balls and you might have to just use rects of 2.
# That is ok.
# Add a new Path to this list for each image.
PATHS = []

# TODO: fill out the other constants below as you are writing the code in functions
# galactic_search_path and _pct_yellow

# TODO: figure out the bounds for filtering just like in the video for the red hat.
# Instead of how the person in the video figured them out, run `./img_viewer.py --hsv <image_file>`
# to view the images in hsv.
# Then, move your cursor around the image and it will display the hue, saturation, and value
# of the pixel you are hovering over. Record the mininum and maximum h, s, and v of all the balls
# in all photos here.
LOWER_YELLOW = np.array([0, 0, 0], dtype=np.uint8)
HIGHER_YELLOW = np.array([255, 255, 255], dtype=np.uint8)

# TODO: once you get to the eroding/dilating step below,
# tune the kernel by trying different sizes (3, 5 ,7).
# You can see if your kernel erodes and dilates properly,
# because when you run the test it will write the image to test_<alliance>_<letter>.png
# which you can view using img_viewer.py
# If needed, you can also use different kernels for eroding and dilating.
KERNEL = np.ones((0, 0), np.uint8)

# Portion of yellow in a rectangle (0 to 1) required for it to be considered as containing a ball.
# TODO: Try different values for this until it correctly reflects whether a ball is in an rectangle
# or not.
BALL_PCT_THRESHOLD = 0


def galactic_search_path(img_path):
    # TODO: read image from img_path into the img variable
    img = None

    # TODO: convert img into hsv
    hsv = None

    # TODO: filter yellow using your bounds for yellow and cv.inRange, creating a binary mask
    mask = None

    # TODO: erode and dilate the mask, and maybe try different numbers of iterations
    mask = None
    mask = None

    correct_path = None
    for path in PATHS:
        # TODO: If all the percentages are atleast BALL_PCT_THRESHOLD,
        # then you can say that this path is present on the field and store it.
        pcts = _pct_yellow(mask, path.rects)

    # TODO: make sure that a path was found, and if not
    # make sure that correct_path has Alliance.UNKNOWN

    return mask, correct_path


# This function finds the percentage of yellow pixels in the rectangles
# given that are regions of the given image. This allows us to determine
# whether there is a ball in those rectangles
def _pct_yellow(mask, rects):
    pcts = np.zeros(len(rects))
    for i in range(len(rects)):
        # TODO: set pcts[i] to be the ratio of the number of yellow pixels in the current rectangle
        # to the total number of pixels in it.
        # You can take the section of the mask that is the rectangle, and then count the number of pixels
        # that aren't zero there with np.count_nonzero to do so,
        # since mask is a 2d array of either 0 or 255.
        pass

    return pcts
