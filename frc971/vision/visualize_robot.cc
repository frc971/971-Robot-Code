#include "frc971/vision/visualize_robot.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "glog/logging.h"

namespace frc971 {
namespace vision {

void VisualizeRobot::SetDefaultViewpoint(int image_width, double focal_length) {
  // 10 meters above the origin, rotated so the camera faces straight down
  Eigen::Translation3d camera_trans(0, 0, 10.0);
  Eigen::AngleAxisd camera_rot(M_PI, Eigen::Vector3d::UnitX());
  Eigen::Affine3d camera_viewpoint = camera_trans * camera_rot;
  SetViewpoint(camera_viewpoint);

  cv::Mat camera_mat;
  double half_width = static_cast<double>(image_width) / 2.0;
  double intr[] = {focal_length, 0.0, half_width, 0.0, focal_length,
                   half_width,   0.0, 0.0,        1.0};
  camera_mat = cv::Mat(3, 3, CV_64FC1, intr);
  SetCameraParameters(camera_mat);

  cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64F, 0.0);
  SetDistortionCoefficients(dist_coeffs);
}

cv::Point VisualizeRobot::ProjectPoint(Eigen::Vector3d T_world_point) {
  // Map 3D point in world coordinates to camera frame
  Eigen::Vector3d T_camera_point = H_world_viewpoint_.inverse() * T_world_point;

  cv::Vec3d T_camera_point_cv;
  cv::eigen2cv(T_camera_point, T_camera_point_cv);

  // Project 3d point in camera frame via camera intrinsics
  cv::Mat proj_point = camera_mat_ * cv::Mat(T_camera_point_cv);
  cv::Point projected_point(
      proj_point.at<double>(0, 0) / proj_point.at<double>(0, 2),
      proj_point.at<double>(0, 1) / proj_point.at<double>(0, 2));
  return projected_point;
}

void VisualizeRobot::DrawLine(Eigen::Vector3d start3d, Eigen::Vector3d end3d,
                              cv::Scalar color) {
  cv::Point start2d = ProjectPoint(start3d);
  cv::Point end2d = ProjectPoint(end3d);

  cv::line(image_, start2d, end2d, color);
}

void VisualizeRobot::DrawFrameAxes(Eigen::Affine3d H_world_target,
                                   std::string label, cv::Scalar label_color,
                                   double axis_scale) {
  // Map origin to display from global (world) frame to camera frame
  Eigen::Affine3d H_viewpoint_target =
      H_world_viewpoint_.inverse() * H_world_target;

  // Extract into OpenCV vectors
  cv::Mat H_viewpoint_target_mat;
  cv::eigen2cv(H_viewpoint_target.matrix(), H_viewpoint_target_mat);

  // Convert to opencv R, T for using drawFrameAxes
  cv::Vec3d rvec, tvec;
  tvec = H_viewpoint_target_mat(cv::Rect(3, 0, 1, 3));
  cv::Mat r_mat = H_viewpoint_target_mat(cv::Rect(0, 0, 3, 3));
  cv::Rodrigues(r_mat, rvec);

  cv::drawFrameAxes(image_, camera_mat_, dist_coeffs_, rvec, tvec, axis_scale);

  if (label != "") {
    // Grab x axis direction
    cv::Vec3d label_offset = r_mat.col(0);

    // Find 3D coordinate of point at the end of the x-axis, and put label there
    // Bump it just a few (5) pixels to the right, to make it read easier
    cv::Mat label_coord_res =
        camera_mat_ * cv::Mat(tvec + label_offset * axis_scale);
    cv::Vec3d label_coord = label_coord_res.col(0);
    label_coord[0] = label_coord[0] / label_coord[2] + 5;
    label_coord[1] = label_coord[1] / label_coord[2];
    cv::putText(image_, label, cv::Point(label_coord[0], label_coord[1]),
                cv::FONT_HERSHEY_PLAIN, 1.0, label_color);
  }
}

void VisualizeRobot::DrawRobotOutline(Eigen::Affine3d H_world_robot,
                                      std::string label, cv::Scalar color) {
  DrawFrameAxes(H_world_robot, label, color);
  const double kBotHalfWidth = 0.75 / 2.0;
  const double kBotHalfLength = 1.0 / 2.0;
  // Compute coordinates for front/rear and right/left corners
  Eigen::Vector3d fr_corner =
      H_world_robot * Eigen::Vector3d(kBotHalfLength, kBotHalfWidth, 0);
  Eigen::Vector3d fl_corner =
      H_world_robot * Eigen::Vector3d(kBotHalfLength, -kBotHalfWidth, 0);
  Eigen::Vector3d rl_corner =
      H_world_robot * Eigen::Vector3d(-kBotHalfLength, -kBotHalfWidth, 0);
  Eigen::Vector3d rr_corner =
      H_world_robot * Eigen::Vector3d(-kBotHalfLength, kBotHalfWidth, 0);

  DrawLine(fr_corner, fl_corner, color);
  DrawLine(fl_corner, rl_corner, color);
  DrawLine(rl_corner, rr_corner, color);
  DrawLine(rr_corner, fr_corner, color);
}

}  // namespace vision
}  // namespace frc971
