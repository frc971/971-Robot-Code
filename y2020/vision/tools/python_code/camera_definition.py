import copy
import glog
import json
import math
import numpy as np
import os

import define_training_data as dtd

glog.setLevel("WARN")

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


def load_camera_definitions():
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
    web_cam_int.camera_matrix = np.asarray([[fx, 0, cx], [0, fy, cy],
                                            [0, 0, 1]])
    web_cam_int.dist_coeffs = np.zeros((5, 1))

    web_cam_ext = CameraExtrinsics()
    # Camera rotation from robot x,y,z to opencv (z, -x, -y)
    # This is extrinsics for the turret camera
    # camera pose relative to center, base of the turret
    # TODO<Jim>: Need to implement per-camera calibration, like with intrinsics
    camera_pitch = 34.0 * np.pi / 180.0
    camera_pitch_matrix = np.matrix(
        [[np.cos(camera_pitch), 0.0, -np.sin(camera_pitch)], [0.0, 1.0, 0.0],
         [np.sin(camera_pitch), 0.0,
          np.cos(camera_pitch)]])
    web_cam_ext.R = np.array(camera_pitch_matrix * np.matrix(
        [[0., 0., 1.], [-1, 0, 0], [0, -1., 0]]))
    # Tape measure calibration-- need to pull from CAD or automate this
    web_cam_ext.T = np.array([2.0 * 0.0254, -6.0 * 0.0254, 41.0 * 0.0254])

    web_cam_params = CameraParameters()
    web_cam_params.camera_int = web_cam_int
    web_cam_params.camera_ext = web_cam_ext

    camera_list = []

    # TODO<Jim>: Should probably make this a dict to make replacing easier
    for team_number in (971, 7971, 8971, 9971):
        for node_name in ("pi0", "pi1", "pi2", "pi3", "pi4", "pi5"):
            camera_base = copy.deepcopy(web_cam_params)
            camera_base.node_name = node_name
            camera_base.team_number = team_number
            camera_list.append(camera_base)

    dir_name = dtd.bazel_name_fix('calib_files')
    for filename in os.listdir(dir_name):
        if "cam-calib-int" in filename and filename.endswith(".txt"):
            # Extract intrinsics from file
            fn_split = filename.split("_")
            hostname_split = fn_split[1].split("-")
            if hostname_split[0] == "pi":
                team_number = int(hostname_split[1])
                node_name = hostname_split[0] + hostname_split[2]

            calib_file = open(dir_name + "/" + filename, 'r')
            calib_dict = json.loads(calib_file.read())
            hostname = np.asarray(calib_dict["hostname"])
            camera_matrix = np.asarray(calib_dict["camera_matrix"])
            dist_coeffs = np.asarray(calib_dict["dist_coeffs"])

            # Look for match, and replace camera_intrinsics
            for camera_calib in camera_list:
                if camera_calib.node_name == node_name and camera_calib.team_number == team_number:
                    glog.info("Found calib for %s, team #%d" % (node_name,
                                                                team_number))
                    camera_calib.camera_int.camera_matrix = copy.copy(
                        camera_matrix)
                    camera_calib.camera_int.dist_coeffs = copy.copy(
                        dist_coeffs)

    return camera_list
