#!/usr/bin/python3

import unittest
import cv2 as cv

import codelab


# TODO(milind): this should be integrated with bazel
class TestCodelab(unittest.TestCase):

    def codelab_test(self, alliance, letter=None, img_path=None):
        if img_path is None:
            img_path = "%s_%s.png" % (alliance.name.lower(),
                                      letter.name.lower())
        mask, path = codelab.galactic_search_path(img_path)

        cv.imwrite("test_" + img_path, mask)

        self.assertEqual(path.alliance, alliance)
        if letter is not None:
            self.assertEqual(path.letter, letter)

    def test_red_a(self):
        self.codelab_test(codelab.Alliance.RED, codelab.Letter.A)

    def test_red_b(self):
        self.codelab_test(codelab.Alliance.RED, codelab.Letter.B)

    def test_blue_a(self):
        self.codelab_test(codelab.Alliance.BLUE, codelab.Letter.A)

    def test_blue_b(self):
        self.codelab_test(codelab.Alliance.BLUE, codelab.Letter.B)

    def test_unknown_path(self):
        """ Makes sure that Alliance.UNKNOWN is returned when there aren't balls in an image """
        self.codelab_test(codelab.Alliance.UNKNOWN, img_path="unknown.png")


if __name__ == "__main__":
    unittest.main()
