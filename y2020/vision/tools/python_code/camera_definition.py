import copy
import math
import numpy as np


class CameraIntrinsics:
    def __init__(self):
        self.camera_matrix = []
        self.dist_coeffs = []

    pass


class CameraExtrinsics:
    def __init__(self):
        self.R = []
        self.T = []


class CameraParameters:
    def __init__(self):
        self.camera_int = CameraIntrinsics()
        self.camera_ext = CameraExtrinsics()
        self.node_name = ""
        self.team_number = -1


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
web_cam_int.dist_coeffs = np.zeros((5, 1))

web_cam_ext = CameraExtrinsics()
# Camera rotation from robot x,y,z to opencv (z, -x, -y)
web_cam_ext.R = np.array([[0., 0., 1.], [-1, 0, 0], [0, -1., 0]])
web_cam_ext.T = np.array([0., 0., 0.])

web_cam_params = CameraParameters()
web_cam_params.camera_int = web_cam_int
web_cam_params.camera_ext = web_cam_ext

camera_list = []

for team_number in (971, 7971, 8971, 9971):
    for node_name in ("pi0", "pi1", "pi2", "pi3", "pi4"):
        camera_base = copy.deepcopy(web_cam_params)
        camera_base.node_name = node_name
        camera_base.team_number = team_number
        camera_list.append(camera_base)
