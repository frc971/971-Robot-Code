import copy
import glog
import json
import math
import numpy as np
import os

import define_training_data as dtd

glog.setLevel("INFO")


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
        self.turret_ext = None
        self.node_name = ""
        self.team_number = -1
        self.timestamp = 0


def load_camera_definitions():
    ### CAMERA DEFINITIONS
    # We only load in cameras that have a calibration file
    # These are stored in y2020/vision/tools/python_code/calib_files
    # See reference_calibration_pi_cam.json for an example default file
    #
    # Or better yet, use //y2020/vision:calibration to calibrate the camera
    #   using a Charuco target board

    # Extrinsic definition
    # Camera rotation from robot x,y,z to opencv (z, -x, -y)
    # This is extrinsics for the turret camera
    # camera pose relative to center, base of the turret
    # TODO<Jim>: Need to implement per-camera calibration, like with intrinsics
    camera_pitch = 34.0 * np.pi / 180.0
    camera_pitch_matrix = np.matrix(
        [[np.cos(camera_pitch), 0.0, -np.sin(camera_pitch)], [0.0, 1.0, 0.0],
         [np.sin(camera_pitch), 0.0,
          np.cos(camera_pitch)]])
    turret_cam_ext = CameraExtrinsics()
    turret_cam_ext.R = np.array(
        camera_pitch_matrix *
        np.matrix([[0., 0., 1.], [-1, 0, 0], [0, -1., 0]]))
    turret_cam_ext.T = np.array([15.0 * 0.0254, 15.0 * 0.0254, 41.0 * 0.0254])
    default_cam_ext = CameraExtrinsics()
    default_cam_ext.R = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0],
                                  [0.0, 0.0, 1.0]])
    default_cam_ext.T = np.array([0.0, 0.0, 0.0])

    default_cam_params = CameraParameters()
    # Currently, all cameras have this same set of extrinsics
    default_cam_params.camera_ext = default_cam_ext
    default_cam_params.turret_ext = turret_cam_ext

    camera_list = []

    dir_name = dtd.bazel_name_fix('calib_files')
    glog.debug("Searching for calibration files in " + dir_name)
    for filename in os.listdir(dir_name):
        glog.debug("Inspecting %s", filename)
        if ("cam-calib-int" in filename
                or 'calibration' in filename) and filename.endswith(".json"):

            # Extract intrinsics from file
            calib_file = open(dir_name + "/" + filename, 'r')
            calib_dict = json.loads(calib_file.read())

            # We have 2 json formats.
            #  1) is Jim's custom format.
            #  2) is the flatbuffer definition of the intrinsics converted to
            #     JSON.
            # See which one we have and parse accordingly.
            if 'hostname' in calib_dict:
                glog.warn("WARNING: Using older calibration file.")
                glog.warn("Preferred usage is y2020/vision:calibration")
                hostname_split = calib_dict["hostname"].split("-")
                team_number = int(hostname_split[1])
                node_name = hostname_split[0] + hostname_split[2]
                camera_matrix = np.asarray(calib_dict["camera_matrix"])
                dist_coeffs = np.asarray(calib_dict["dist_coeffs"])
            else:
                team_number = calib_dict["team_number"]
                node_name = calib_dict["node_name"]
                camera_matrix = np.asarray(calib_dict["intrinsics"]).reshape(
                    (3, 3))
                dist_coeffs = np.asarray(calib_dict["dist_coeffs"]).reshape(
                    (1, 5))

            glog.info("Found calib for " + node_name + ", team #" +
                      str(team_number))
            camera_base = copy.deepcopy(default_cam_params)
            camera_base.node_name = node_name
            camera_base.team_number = team_number
            camera_base.camera_int.camera_matrix = copy.copy(camera_matrix)
            camera_base.camera_int.dist_coeffs = copy.copy(dist_coeffs)
            camera_list.append(camera_base)

    return camera_list
