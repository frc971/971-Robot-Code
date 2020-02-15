import cv2
import math
from matplotlib import pyplot as plt
import numpy as np

import field_display

# Assume image is 640x480 (VGA)
num_pixels_x = 640
num_pixels_y = 480

# Camera center is 320, 240
c_x = num_pixels_x / 2
c_y = num_pixels_y / 2

# Horiz. FOV is 54 degrees
FOV_h = 54 * math.pi / 180.0  # (in radians)
# Vert FOV is horizontal FOV scaled by aspect ratio (through tan function)
FOV_v = math.atan(c_y / c_x * math.tan(FOV_h / 2.)) * 2

# Assuming square pixels
# focal length is (640/2)/tan(FOV_h/2)
f_x = c_x / (math.tan(FOV_h / 2.))
f_y = c_y / (math.tan(FOV_v / 2.))

# Height of camera on robot above ground
cam_above_ground = 0.5

# TODO: Should fix f_y entry below.
cam_mat = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1.]])

# Use default distortion (i.e., none)
distortion_coeffs = np.zeros((5, 1))

depth = 5.0  # meters

R_w_tarj_mat = np.array([[0., 0., 1.], [-1, 0, 0], [0, -1., 0]])
R_w_tarj, jac = cv2.Rodrigues(R_w_tarj_mat)
T_w_tarj = np.array([[15.98 - depth, -4.10 + 2.36, cam_above_ground]]).T

img_ret = field_display.plot_bot_on_field(None, (0, 255, 0), T_w_tarj)
#field_display.show_field(img_ret)

# Create fake set of points relative to camera capture (target) frame
# at +/- 1 meter in x, 5 meter depth, and every 1 meter in y from 0 to 4m (above the camera, so negative y values)
pts_3d_T_t = np.array([[-1., 0., depth], [1., 0., depth], [-1., -1., depth], [
    1., -1., depth
], [-1., -2., depth], [0., -2., depth], [1., -2., depth], [-1., -3., depth],
                       [1., -3., depth], [-1., -4., depth], [1., -4., depth]])

# Ground truth shift of camera, to compute projections
R_tarj_ci_gt = np.array([[0., 0.2, 0.2]]).T

R_tarj_ci_gt_mat, jac = cv2.Rodrigues(R_tarj_ci_gt)

T_tarj_ci_gt = np.array([[1., 2., -5.]]).T

# To transform vectors, we apply the inverse transformation
R_tarj_ci_gt_mat_inv = R_tarj_ci_gt_mat.T
T_tarj_ci_gt_inv = -R_tarj_ci_gt_mat_inv.dot(T_tarj_ci_gt)

#pts_3d_T_t_shifted = (R_tarj_ci_gt_mat_inv.dot(pts_3d_T_t.T) + T_tarj_ci_gt_inv).T

# Now project from new position
# This was the manual way to do it-- use cv2.projectPoints instead
#pts_proj_3d = cam_mat.dot(pts_3d_T_t_shifted.T).T
#pts_proj_2d = np.divide(pts_proj_3d[:,0:2],(pts_proj_3d[:,2].reshape(-1,1)))

pts_proj_2d_ci, jac_2d = cv2.projectPoints(pts_3d_T_t, R_tarj_ci_gt_mat_inv,
                                           T_tarj_ci_gt_inv, cam_mat,
                                           distortion_coeffs)
#print(pts_proj_2d_ci)

# Now, solve for the pose using the original 3d points (pts_3d_T_t) and the projections from the new location
retval, R_ci_tarj_est, T_ci_tarj_est, inliers = cv2.solvePnPRansac(
    pts_3d_T_t, pts_proj_2d_ci, cam_mat, distortion_coeffs)

# This is the pose from camera to original target spot.  We need to invert to get back to the pose we want
R_tarj_ci_est_mat = cv2.Rodrigues(R_ci_tarj_est)[0].T
T_tarj_ci_est = -R_tarj_ci_est_mat.dot(T_ci_tarj_est)
R_tarj_ci_est = cv2.Rodrigues(R_tarj_ci_est_mat)[0]

print("Check:\n", "Rot error:\n", R_tarj_ci_gt - R_tarj_ci_est,
      "\nTrans error:\n", T_tarj_ci_gt - T_tarj_ci_est,
      "\nError is < 0.001 in R & T: ",
      np.linalg.norm(R_tarj_ci_gt - R_tarj_ci_est) < 0.001,
      np.linalg.norm(T_tarj_ci_gt - T_tarj_ci_est) < 0.001)

# Compute camera location in world coordinates
R_w_ci, T_w_ci, d1, d2, d3, d4, d5, d6, d7, d8 = cv2.composeRT(
    R_tarj_ci_est, T_tarj_ci_est, R_w_tarj, T_w_tarj)

print("Estimate in world coordinates")
print("R, T:\n", R_w_ci, "\n", T_w_ci)
img_ret = field_display.plot_bot_on_field(img_ret, (0, 255, 255), T_w_ci)
field_display.show_field(img_ret)

# Compute vector to target
# TODO: Likely better to do this from the homography, rather than the pose estimate...

T_w_target_pt = np.array([[15.98, -4.10 + 2.36, 2.0 - cam_above_ground]]).T
vector_to_target = T_w_target_pt - T_w_ci
d_ci_target = np.linalg.norm(vector_to_target)
phi_ci_target = math.atan2(vector_to_target[1][0], vector_to_target[0][0])
print("Vector to target (from cam frame):\n", vector_to_target,
      "\nDistance to target: ", d_ci_target, "\nAngle to target (deg): ",
      phi_ci_target * 180. / math.pi)

img_ret = field_display.plot_line_on_field(img_ret, (255, 255, 0), T_w_ci,
                                           T_w_target_pt)
field_display.show_field(img_ret)
