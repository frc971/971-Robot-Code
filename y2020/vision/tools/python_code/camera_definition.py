import argparse
import cv2
import json
import math
import numpy as np
import time


class CameraIntrinsics:
    def __init__(self):
        self.camera_matrix = []
        self.distortion_coeffs = []

    pass

class CameraExtrinsics:
    def __init__(self):
        self.R = []
        self.T = []

    pass

class CameraParameters:
    def __init__(self):
        self.camera_int = CameraIntrinsics()
        self.camera_ext = CameraExtrinsics()

    pass


### CAMERA DEFINITIONS

# Robot camera has:
# FOV_H = 93.*math.pi()/180.
# FOV_V = 70.*math.pi()/180.

# Create fake camera (based on USB webcam params)
fx = 810.
fy = 810.
cx = 320.
cy = 240.

# Define a web_cam
web_cam_int = CameraIntrinsics()
web_cam_int.camera_matrix = np.asarray([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
web_cam_int.distortion_coeffs = np.zeros((5,1))

web_cam_ext = CameraExtrinsics()
# Camera rotation from robot x,y,z to opencv (z, -x, -y)
web_cam_ext.R = np.array([[0., 0., 1.],
                          [-1, 0,  0],
                          [0, -1., 0]])
web_cam_ext.T = np.array([[0., 0., 0.]]).T

web_cam_params = CameraParameters()
web_cam_params.camera_int = web_cam_int
web_cam_params.camera_ext = web_cam_ext
