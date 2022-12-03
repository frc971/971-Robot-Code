#include "frc971/vision/visualize_robot.h"
#include "glog/logging.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace frc971 {
namespace vision {

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

void VisualizeRobot::DrawLine(Eigen::Vector3d start3d, Eigen::Vector3d end3d) {
  cv::Point start2d = ProjectPoint(start3d);
  cv::Point end2d = ProjectPoint(end3d);

  cv::line(image_, start2d, end2d, cv::Scalar(0, 0, 255));
}

void VisualizeRobot::DrawFrameAxes(Eigen::Affine3d H_world_target,
                                   std::string label, double axis_scale) {
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

    // Find 3D coordinate of point at the end of the x-axis, and project it
    cv::Mat label_coord_res =
        camera_mat_ * cv::Mat(tvec + label_offset * axis_scale);
    cv::Vec3d label_coord = label_coord_res.col(0);
    label_coord[0] = label_coord[0] / label_coord[2];
    label_coord[1] = label_coord[1] / label_coord[2];
    cv::putText(image_, label, cv::Point(label_coord[0], label_coord[1]),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
  }
}

void VisualizeRobot::DrawBoardOutline(Eigen::Affine3d H_world_board,
                                      std::string label) {
  LOG(INFO) << "Not yet implemented; drawing axes only";
  DrawFrameAxes(H_world_board, label);
}

}  // namespace vision
}  // namespace frc971
