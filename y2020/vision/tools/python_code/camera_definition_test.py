import copy
import math
import numpy as np

from y2020.vision.tools.python_code.camera_definition import (CameraIntrinsics,
                                                              CameraExtrinsics,
                                                              CameraParameters)

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

for team_number in (971, 8971, 9971):
    for (i, node_name) in enumerate(("pi-1", "pi-2", "pi-3", "pi-4", "pi-5")):
        camera_base = copy.deepcopy(web_cam_params)
        camera_base.node_name = node_name
        camera_base.team_number = team_number
        camera_base.camera_ext.T = np.asarray(np.float32([i + 1, i + 1,
                                                          i + 1]))
        camera_list.append(camera_base)
