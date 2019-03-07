#!/usr/bin/python

import cv2
import glob
import math
import numpy as np
import sys
"""
Usage:
    undistort.py [display]

Finds files in /tmp/*.yuyv to compute distortion constants for.
"""


def undist(orig, mtx, dist, newcameramtx, its=1):
    """
    This function runs a manual undistort over the entire image to compare to the
    golden as proof that the algorithm works and the generated constants are correct.
    """
    output = np.full(orig.shape, 255, dtype=np.uint8)
    for i in range(480):
        for j in range(640):
            x0 = (i - mtx[1, 2]) / mtx[1, 1]
            y0 = (j - mtx[0, 2]) / mtx[0, 0]
            x = x0
            y = y0
            for k in range(its):
                r2 = x * x + y * y
                coeff = (1 + dist[0, 0] * r2 + dist[0, 1] * math.pow(r2, 2) +
                         dist[0, 4] * math.pow(r2, 3))
                x = x0 / coeff
                y = y0 / coeff
            ip = x * newcameramtx[1, 1] + newcameramtx[1, 2]
            jp = y * newcameramtx[0, 0] + newcameramtx[0, 2]
            if ip < 0 or jp < 0 or ip >= 480 or jp >= 640:
                continue
            output[int(ip), int(jp)] = orig[i, j]
    return output


def main(argv):
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/tmp/*.yuyv')

    cols = 640
    rows = 480

    # Iterate through all the available images
    for fname in images:
        fd = open(fname, 'rb')
        f = np.fromfile(fd, np.uint8, cols * rows * 2)
        # Convert yuyv color space to single channel grey.
        grey = f[::2]
        grey = np.reshape(grey, (rows, cols))

        ret, corners = cv2.findChessboardCorners(grey, (9, 6), None)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            # Draw the chessboard with corners marked.
            if len(argv) > 1 and argv[1] == 'display':
                rgb = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
                cv2.drawChessboardCorners(rgb, (9, 6), corners, ret)
                cv2.imshow('', rgb)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        fd.close()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, grey.shape[::-1], None, None)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (rows, cols),
                                                      1, (rows, cols))

    dist[0, 2] = 0
    dist[0, 3] = 0
    print("Formatted for Game Config:")
    print("""distortion {
  f_x: %f
  c_x: %f
  f_y: %f
  c_y :%f
  f_x_prime: %f
  c_x_prime: %f
  f_y_prime: %f
  c_y_prime: %f
  k_1: %f
  k_2: %f
  k_3: %f
  distortion_iterations: 7
}""" % (
        # f_x c_x
        mtx[0][0],
        mtx[0][2],
        # f_y c_y
        mtx[1][1],
        mtx[1][2],
        # f_x c_x prime
        newcameramtx[0][0],
        newcameramtx[0][2],
        # f_y c_y prime
        newcameramtx[1][1],
        newcameramtx[1][2],
        # k_1, k_2, k_3
        dist[0, 0],
        dist[0, 1],
        dist[0, 4]))

    # Draw the original image, open-cv undistort, and our undistort in separate
    # windows for each available image.
    if len(argv) > 1 and argv[1] == 'display':
        for fname in images:
            fd = open(fname, 'rb')
            f = np.fromfile(fd, np.uint8, cols * rows * 2)
            grey_t = f[::2]
            grey_t = np.reshape(grey_t, (rows, cols))
            dst_expected = cv2.undistort(grey_t, mtx, dist, None, newcameramtx)
            dst_actual = undist(grey_t, mtx, dist, newcameramtx, 5)
            cv2.imshow('orig', grey_t)
            cv2.imshow('opencv undistort', dst_expected)
            cv2.imshow('our undistort', dst_actual)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            fd.close()


if __name__ == '__main__':
    main(sys.argv)
