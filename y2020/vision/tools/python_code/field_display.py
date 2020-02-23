import cv2
import math
import numpy as np


def load_field_image():
    field_image = cv2.imread('test_images/field_cropped_scaled.png')
    return field_image


def show_field(img_in=None):
    if (img_in is None):
        img_in = load_field_image

    cv2.imshow('Field', img_in), cv2.waitKey()
    return


# Convert field coordinates (meters) to image coordinates (pixels).
# Field origin is (x,y) at center of the field,
# with x pointing towards the red alliance driver station
# The default field image is 1598 x 821 pixels, so 1 cm per pixel on field
# I.e., Field is 1598 x 821 pixels = 15.98 x 8.21 meters
# Our origin in image coordinates is at (799, 410) pixels, with +x direction
# to the right


def field_coord_to_image_coord(robot_position):
    # Scale by 100 pixel / cm
    robot_pos_img_coord = np.array([[799, 410]]).T + np.int32(
        100.0 * np.array([[robot_position[0][0], -robot_position[1][0]]]).T)
    return robot_pos_img_coord


# Given a field image, plot the robot and an optional heading vector on it
def plot_bot_on_field(img_in, color, robot_position, robot_heading=None):
    if img_in is None:
        img_out = load_field_image()
    else:
        img_out = img_in.copy()

    robot_pos_img_coord = field_coord_to_image_coord(robot_position)

    ROBOT_RADIUS = 10
    img_out = cv2.circle(
        img_out, (robot_pos_img_coord[0][0], robot_pos_img_coord[1][0]),
        ROBOT_RADIUS, color, 3)

    if (robot_heading is not None):
        img_out = cv2.line(
            img_out, (robot_pos_img_coord[0][0], robot_pos_img_coord[1][0]),
            (int(robot_pos_img_coord[0][0] -
                 3 * ROBOT_RADIUS * math.cos(robot_heading)),
             int(robot_pos_img_coord[1][0] +
                 3 * ROBOT_RADIUS * math.sin(robot_heading))), color, 3,
            cv2.LINE_AA)

    return img_out


# Plot a line on the field, given starting and ending field positions
def plot_line_on_field(img_in, color, start_position, end_position):
    if img_in is None:
        img_out = load_field_image()
    else:
        img_out = img_in.copy()

    start_pos_img_coord = field_coord_to_image_coord(start_position)
    end_pos_img_coord = field_coord_to_image_coord(end_position)

    img_out = cv2.line(img_out,
                       (start_pos_img_coord[0][0], start_pos_img_coord[1][0]),
                       (end_pos_img_coord[0][0], end_pos_img_coord[1][0]),
                       color, 3, cv2.LINE_AA)

    return img_out


# Helper function to plot a few quantities like
#   Heading (angle of the target relative to the robot)
#   Distance (of the target to the robot)
#   Skew (angle of the target normal relative to the robot)
def plot_camera_to_target(img_in, color, heading, distance, skew):
    if img_in is None:
        img_out = np.zeros((821, 1598, 3), np.uint8)
    else:
        img_out = img_in

    # Bot location is at origin
    bot_location = np.array([[0., 0.]]).T

    # Target location is distance away along the heading
    target_location = bot_location + np.array(
        [[distance * math.cos(heading), distance * math.sin(heading)]]).T

    # Create part of a line from the target location to the end of the target, based on the skew
    skew_line_offset = np.array(
        [[math.cos(skew + math.pi / 2),
          math.sin(skew + math.pi / 2)]]).T

    # Plot bot origin
    img_out = plot_bot_on_field(img_out, color, bot_location)

    # Plot heading line
    img_out = plot_line_on_field(img_out, (255, 255, 0), bot_location,
                                 target_location)

    # Plot target location
    img_out = plot_bot_on_field(img_out, (255, 0, 0), target_location)

    # Plot target rotation
    img_out = plot_line_on_field(img_out, (0, 255, 0),
                                 target_location - skew_line_offset,
                                 target_location + skew_line_offset)

    return img_out


# Helper function to take camera rotation and bring it back to world coordinate frame
def camera_rot_to_world(R_mat):
    R_c_w_mat = np.array([[0., 0., 1.], [-1, 0, 0], [0, -1., 0]])

    return (R_mat).dot(R_c_w_mat.T)


def camera_rot_to_world_Rodrigues(R):
    R_mat = cv2.Rodrigues(R)[0]
    R_mat_world = camera_rot_to_world(R_mat)
    R_world = cv2.Rodrigues(R_mat_world)[0]
    return R_world


def invert_pose_Rodrigues(R, T):
    R_inv_mat = cv2.Rodrigues(R)[0].T
    T_inv = -R_inv_mat.dot(T)
    R_inv = cv2.Rodrigues(R_inv_mat)[0]
    return R_inv, T_inv


def invert_pose_mat(R, T):
    R_inv = R.T
    T_inv = -R_inv.dot(T)
    return R_inv, T_inv
