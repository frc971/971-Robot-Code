#!/usr/bin/python3

from absl import app, flags
import cv2 as cv
import glog
import matplotlib.pyplot as plt

flags.DEFINE_bool("hsv", False, "Displays the image in hsv")


def main(argv):
    glog.check_eq(len(argv), 2, "Expected one image filename as an argument")
    bgr_img = cv.imread(argv[1])
    img = cv.cvtColor(
        bgr_img, cv.COLOR_BGR2HSV if flags.FLAGS.hsv else cv.COLOR_BGR2RGB)
    plt.imshow(img)
    plt.show()


if __name__ == "__main__":
    app.run(main)
