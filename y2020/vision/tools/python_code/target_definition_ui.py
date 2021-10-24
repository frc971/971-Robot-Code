#!/usr/bin/python3

from absl import app, flags
from datetime import datetime
import glog
import matplotlib.pyplot as plt
import numpy as np

import camera_definition
import target_definition

flags.DEFINE_float("robot_pos_x", 0,
                   "X position of robot relative to center of field")
flags.DEFINE_float("robot_pos_y", 0,
                   "Y position of robot relative to center of field")
flags.DEFINE_float("robot_heading", 0, "Heading angle of the robot in degrees")

flags.DEFINE_string("image_filename", None, "Path to image file to use")
flags.DEFINE_string("input_json_filename", None,
                    "Path to target json file to read 3d points from")
flags.DEFINE_string("output_json_filename", None,
                    "Path to target json file to write to")

FLAGS = flags.FLAGS


def load_pi1_camera_params():
    camera_params = None
    for camera in camera_definition.load_camera_definitions():
        if camera.team_number == 971 and camera.node_name == "pi1":
            camera_params = camera
            break
    glog.check_notnone(camera_params,
                       "Could not find camera params for team 971 pi1")
    return camera_params


_camera_params = load_pi1_camera_params()


def project_to_image_coords(robot_pos_x, robot_pos_y, robot_heading_degrees,
                            point3d):
    """ Given a robot 2D location, project a given 3D point in space to a 2D image coordinate """

    robot_heading = np.radians(robot_heading_degrees)
    # Create a Homogeneous tranform of the Robot position with respect to the world
    H_w_r = np.identity(4, np.double)
    R_w_r = np.array([[np.cos(robot_heading), -np.sin(robot_heading), 0],
                      [np.sin(robot_heading),
                       np.cos(robot_heading), 0], [0, 0, 1]])
    H_w_r[0:3, 0:3] = R_w_r
    H_w_r[0:2, 3] = np.array([robot_pos_x, robot_pos_y])

    # Pull in the extrinsics of the camera from global _camera_params
    H_camera_ext = np.identity(4, np.double)
    global _camera_params
    H_camera_ext[0:3, 0:3] = _camera_params.camera_ext.R
    H_camera_ext[0:3, 3] = _camera_params.camera_ext.T

    # Have to also include the turret-- assuming camera is
    # at 0 rotation of the turret
    H_turret_ext = np.identity(4, np.double)
    H_turret_ext[0:3, 0:3] = _camera_params.turret_ext.R
    H_turret_ext[0:3, 3] = _camera_params.turret_ext.T

    # From those two, calculate the full extrinsics, from robot to camera
    H_r_c = H_camera_ext @ H_turret_ext

    # Extract the camera intrinsics
    cam_int_mat = _camera_params.camera_int.camera_matrix

    # Find the transform that maps the camera to the world frame
    H_c_w = np.linalg.inv(H_w_r @ H_r_c)

    # Create a homogeneous representation of the target point in 3d (a 4x1 vec)
    T_point3d = np.ones(4, np.double)
    T_point3d[0:3] = point3d

    # Project that point into the image plane (normalizing by z value)
    proj_point = cam_int_mat @ (H_c_w @ T_point3d)[0:3]
    proj_point /= proj_point[2]

    return proj_point[0:2]


def draw_circle(ax, td):
    """
    Draws a circle where the next 3d point should be in the image.
    The user clicks on the polygon point on the image that is closest to
    the circle, and then that point is appended to td.polygon_list[0].
    We know that the current 3d point in td.polygon_list_3d[0]
    has index len(td.polygon_list[0]) because of this.
    We are using 2d arrays of points and only touching the first row in order to support
    multiple polygons per target in the future.
    """
    point2d = project_to_image_coords(
        FLAGS.robot_pos_x, FLAGS.robot_pos_y, FLAGS.robot_heading,
        td.polygon_list_3d[0][len(td.polygon_list[0])])
    ax.add_patch(plt.Circle(point2d, 10, color="yellow", fill=False))


_on_click_id = None


def on_click(event, fig, ax, timer, text, td):
    point2d = [round(event.xdata), round(event.ydata)]
    # They clicked on the target point after finishing polygon points
    if len(td.polygon_list[0]) == len(td.polygon_list_3d[0]):
        # Disconnect the on click so that nothing gets called if the user
        # clicks during the short timeout to show the done message
        global _on_click_id
        fig.canvas.mpl_disconnect(_on_click_id)

        td.target_point_2d = np.float32([point2d]).reshape(-1, 1, 2)
        text.set_size(8)
        text.set_text("Done capturing points.\nSaving as file " +
                      FLAGS.output_json_filename)

        timer.add_callback(lambda: plt.close("all"))
        timer.start()
    else:
        ax.patches.clear()
        td.polygon_list[0].append(point2d)
        if len(td.polygon_list[0]) == len(td.polygon_list_3d[0]):
            text.set_text("Click on target point")
        else:
            text.set_text(
                "Click on point %d out of %d" %
                (len(td.polygon_list[0]) + 1, len(td.polygon_list_3d[0])))
            draw_circle(ax, td)
    plt.draw()


def main(_):
    """
    Runs a UI that lets the user click on points in an image of a target.
    Writes these points to a json so they can be used for target definitions.
    """

    ############################################################
    # TARGET DEFINITIONS
    ############################################################

    # Using coordinate system as defined in sift.fbs:
    # field origin (0, 0, 0) at floor height at center of field
    # Robot orientation with x-axis pointing towards RED ALLIANCE player station
    # y-axis to left and z-axis up (right-hand coordinate system)
    # Thus, the red power port target location will be at (-15.98/2,1.67,0),
    # with orientation (facing out of the field) of M_PI

    td = target_definition.TargetData.from_json(FLAGS.input_json_filename)
    td.image_filename = FLAGS.image_filename
    td.polygon_list = [[]]

    if not FLAGS.output_json_filename:
        FLAGS.output_json_filename = "%s_%s.json" % (
            FLAGS.input_json_filename[:-len(".json")],
            datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    fig, ax = plt.subplots()
    ax.imshow(plt.imread(td.image_filename))
    text = ax.text(0,
                   0,
                   "Click on point 1 out of %d" % len(td.polygon_list_3d[0]),
                   size=10,
                   backgroundcolor="white")
    draw_circle(ax, td)

    timer = fig.canvas.new_timer(interval=2000)
    global _on_click_id
    _on_click_id = fig.canvas.mpl_connect(
        "button_press_event",
        lambda event: on_click(event, fig, ax, timer, text, td))
    plt.show()

    glog.info("Done configuring points, writing to %s",
              FLAGS.output_json_filename)
    td.write_to_json(FLAGS.output_json_filename)


if __name__ == "__main__":
    flags.mark_flags_as_required(["image_filename", "input_json_filename"])
    app.run(main)
