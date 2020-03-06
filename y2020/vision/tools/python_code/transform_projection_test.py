import cv2
import math
import numpy as np

import field_display
import camera_definition

# Import camera from camera_definition
camera_params = camera_definition.web_cam_params
cam_mat = camera_params.camera_int.camera_matrix
dist_coeffs = camera_params.camera_int.dist_coeffs

# Height of camera on robot above ground
cam_above_ground = 0.5
depth = 5.0  # meters

# Define camera location (cam) relative to origin (w)
R_w_cam_mat = np.array([[0., 0., 1.], [-1, 0, 0], [0, -1., 0]])
R_w_cam, jac = cv2.Rodrigues(R_w_cam_mat)
T_w_cam = np.array([[15.98 - depth, -4.10 + 2.36, cam_above_ground]]).T

img_ret = field_display.plot_bot_on_field(None, (0, 255, 0), T_w_cam)

# Create fake set of points relative to camera capture (target) frame
# at +/- 1 meter in x, 5 meter depth, and every 1 meter in y from 0 to 4m (above the camera, so negative y values)
pts_3d_target = np.array(
    [[-1., 0., depth], [1., 0., depth], [-1., -1., depth], [1., -1., depth],
     [-1., -2., depth], [0., -2., depth], [1., -2., depth], [-1., -3., depth],
     [1., -3., depth], [-1., -4., depth], [1., -4., depth]])

# Ground truth shift of camera from (cam) to (cam2), to compute projections
R_cam_cam2_gt = np.array([[0., 0.2, 0.2]]).T

R_cam_cam2_gt_mat, jac = cv2.Rodrigues(R_cam_cam2_gt)

T_cam_cam2_gt = np.array([[1., 2., -5.]]).T

# To transform vectors, we apply the inverse transformation
R_cam_cam2_gt_mat_inv = R_cam_cam2_gt_mat.T
T_cam_cam2_gt_inv = -R_cam_cam2_gt_mat_inv.dot(T_cam_cam2_gt)

#pts_3d_target_shifted = (R_cam_cam2_gt_mat_inv.dot(pts_3d_target.T) + T_cam_cam2_gt_inv).T

# Now project from new position
# This was the manual way to do it-- use cv2.projectPoints instead
#pts_proj_3d = cam_mat.dot(pts_3d_target_shifted.T).T
#pts_proj_2d = np.divide(pts_proj_3d[:,0:2],(pts_proj_3d[:,2].reshape(-1,1)))

pts_proj_2d_cam2, jac_2d = cv2.projectPoints(
    pts_3d_target, R_cam_cam2_gt_mat_inv, T_cam_cam2_gt_inv, cam_mat,
    dist_coeffs)

# Now, solve for the pose using the original 3d points (pts_3d_T_t) and the projections from the new location
retval, R_cam2_cam_est, T_cam2_cam_est, inliers = cv2.solvePnPRansac(
    pts_3d_target, pts_proj_2d_cam2, cam_mat, dist_coeffs)

# This is the pose from camera to original target spot.  We need to invert to get back to the pose we want
R_cam_cam2_est_mat = cv2.Rodrigues(R_cam2_cam_est)[0].T
T_cam_cam2_est = -R_cam_cam2_est_mat.dot(T_cam2_cam_est)
R_cam_cam2_est = cv2.Rodrigues(R_cam_cam2_est_mat)[0]

print("Check:\n", "Rot error:\n", R_cam_cam2_gt - R_cam_cam2_est,
      "\nTrans error:\n", T_cam_cam2_gt - T_cam_cam2_est,
      "\nError is < 0.001 in R & T: ",
      np.linalg.norm(R_cam_cam2_gt - R_cam_cam2_est) < 0.001, " & ",
      np.linalg.norm(T_cam_cam2_gt - T_cam_cam2_est) < 0.001)

# Compute camera location in world coordinates
R_w_cam2, T_w_cam2, d1, d2, d3, d4, d5, d6, d7, d8 = cv2.composeRT(
    R_cam_cam2_est, T_cam_cam2_est, R_w_cam, T_w_cam)

print("Estimate in world coordinates")
print("R, T:\n", R_w_cam2, "\n", T_w_cam2)
img_ret = field_display.plot_bot_on_field(img_ret, (0, 255, 255), T_w_cam2)
field_display.show_field(img_ret)

# Compute vector to target
# TODO: Likely better to do this from the homography, rather than the pose estimate...

T_w_target_pt = np.array([[15.98, -4.10 + 2.36, 2.0 - cam_above_ground]]).T
vector_to_target = T_w_target_pt - T_w_cam2
d_cam2_target = np.linalg.norm(vector_to_target)
phi_cam2_target = math.atan2(vector_to_target[1][0], vector_to_target[0][0])
print("Vector to target (from cam frame):\n", vector_to_target,
      "\nDistance to target: ", d_cam2_target, "\nAngle to target (deg): ",
      phi_cam2_target * 180. / math.pi)

img_ret = field_display.plot_line_on_field(img_ret, (255, 255, 0), T_w_cam2,
                                           T_w_target_pt)
field_display.show_field(img_ret)
