import copy
import glog
import json
import math
import numpy as np
import os

glog.setLevel("INFO")


# Quick fix to naming games that happen with bazel
def bazel_name_fix(filename):
    ret_name = filename
    try:
        from bazel_tools.tools.python.runfiles import runfiles
        r = runfiles.Create()
        ret_name = r.Rlocation('org_frc971/y2022/vision/' + filename)
        #print("Trying directory: ", ret_name)
    except:
        print("Failed bazel_name_fix")
        pass

    ### TODO<Jim>: Need to figure out why this isn't working
    ### Hardcoding for now
    if ret_name == None:
        ret_name = '/home/jim/code/FRC/971-Robot-Code/y2022/vision/calib_files'
    return ret_name


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


def compute_extrinsic(camera_pitch, camera_yaw, T_camera, is_turret):
    # Compute the extrinsic calibration based on pitch and translation
    # Includes camera rotation from robot x,y,z to opencv (z, -x, -y)

    # Also, handle extrinsics for the turret
    # The basic camera pose is relative to the center, base of the turret
    # TODO<Jim>: Maybe store these to .json files, like with intrinsics?
    base_cam_ext = CameraExtrinsics()
    turret_cam_ext = CameraExtrinsics()

    camera_pitch_matrix = np.array(
        [[np.cos(camera_pitch), 0.0,
          np.sin(camera_pitch)], [0.0, 1.0, 0.0],
         [-np.sin(camera_pitch), 0.0,
          np.cos(camera_pitch)]])

    camera_yaw_matrix = np.array(
        [[np.cos(camera_yaw), -np.sin(camera_yaw), 0.0],
         [np.sin(camera_yaw), np.cos(camera_yaw), 0.0], [0.0, 0.0, 1.0]])

    robot_to_camera_rotation = np.array([[0., 0., 1.], [-1, 0, 0], [0, -1.,
                                                                    0]])

    if is_turret:
        # Turret camera has default heading 180 deg away from the robot x
        base_cam_ext.R = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0],
                                   [0.0, 0.0, 1.0]])
        base_cam_ext.T = np.array([0.0, 0.0, 0.0])
        turret_cam_ext.R = camera_yaw_matrix @ camera_pitch_matrix @ robot_to_camera_rotation
        turret_cam_ext.T = T_camera
    else:
        base_cam_ext.R = camera_yaw_matrix @ camera_pitch_matrix @ robot_to_camera_rotation
        base_cam_ext.T = T_camera
        turret_cam_ext = None

    return base_cam_ext, turret_cam_ext


def compute_extrinsic_by_pi(pi_number):
    # Defaults for non-turret camera
    camera_pitch = -20.0 * np.pi / 180.0
    camera_yaw = 0.0
    is_turret = False
    # Default camera location to robot origin
    T = np.array([0.0, 0.0, 0.0])

    if pi_number == "pi1":
        # This is the turret camera
        camera_pitch = -10.0 * np.pi / 180.0
        is_turret = True
        T = np.array([7.5 * 0.0254, -5.5 * 0.0254, 41.0 * 0.0254])
    elif pi_number == "pi2":
        T = np.array([4.5 * 0.0254, 3.75 * 0.0254, 26.0 * 0.0254])
    elif pi_number == "pi3":
        camera_yaw = 90.0 * np.pi / 180.0
        T = np.array([-2.75 * 0.0254, 5.25 * 0.0254, 25.25 * 0.0254])
    elif pi_number == "pi4":
        camera_yaw = -90.0 * np.pi / 180.0
        T = np.array([-2.75 * 0.0254, -6 * 0.0254, 26 * 0.0254])
    elif pi_number == "pi5":
        camera_yaw = 180.0 * np.pi / 180.0
        T = np.array([-7.25 * 0.0254, -4.24 * 0.0254, 16.5 * 0.0254])

    return compute_extrinsic(camera_pitch, camera_yaw, T, is_turret)


def load_camera_definitions():
    ### CAMERA DEFINITIONS
    # We only load in cameras that have a calibration file
    # These are stored in y2022/vision/calib_files
    #
    # Or better yet, use //y2020/vision:calibration to calibrate the camera
    #   using a Charuco target board

    camera_list = []

    dir_name = bazel_name_fix('calib_files')
    if dir_name is not None:
        glog.debug("Searching for calibration files in " + dir_name)
    else:
        glog.fatal("Failed to find calib_files directory")

    for filename in sorted(os.listdir(dir_name)):
        glog.debug("Inspecting %s", filename)
        if ("cam-calib-int" in filename
                or 'calibration' in filename) and filename.endswith(".json"):

            # Extract intrinsics from file
            calib_file = open(dir_name + "/" + filename, 'r')
            calib_dict = json.loads(calib_file.read())

            team_number = calib_dict["team_number"]
            node_name = calib_dict["node_name"]
            camera_matrix = np.asarray(calib_dict["intrinsics"]).reshape(
                (3, 3))
            dist_coeffs = np.asarray(calib_dict["dist_coeffs"]).reshape((1, 5))

            glog.debug("Found calib for " + node_name + ", team #" +
                       str(team_number))

            camera_params = CameraParameters()
            # TODO: Need to add reading in extrinsic camera parameters from json
            camera_params.camera_ext, camera_params.turret_ext = compute_extrinsic_by_pi(
                node_name)

            camera_params.node_name = node_name
            camera_params.team_number = team_number
            camera_params.camera_int.camera_matrix = copy.copy(camera_matrix)
            camera_params.camera_int.dist_coeffs = copy.copy(dist_coeffs)
            camera_list.append(camera_params)

    return camera_list
