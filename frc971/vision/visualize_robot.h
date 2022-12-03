#ifndef FRC971_VISION_VISUALIZE_ROBOT_H_
#define FRC971_VISION_VISUALIZE_ROBOT_H_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace frc971 {
namespace vision {

// Helper class to visualize the coordinate frames associated with
// the robot Based on a virtual camera viewpoint, and camera model,
// this class can be used to draw 3D coordinate frames in a virtual
// camera view.
//
// Mostly useful just for doing all the projection calculations
// Leverages Eigen for transforms and opencv for drawing axes

class VisualizeRobot {
 public:
  // Set image on which to draw
  void SetImage(cv::Mat image) { image_ = image; }

  // Set the viewpoint of the camera relative to a global origin
  void SetViewpoint(Eigen::Affine3d view_origin) {
    H_world_viewpoint_ = view_origin;
  }

  // Set camera parameters (for projection into a virtual view)
  void SetCameraParameters(cv::Mat camera_intrinsics) {
    camera_mat_ = camera_intrinsics;
  }

  // Set distortion coefficients (defaults to 0)
  void SetDistortionCoefficients(cv::Mat dist_coeffs) {
    dist_coeffs_ = dist_coeffs;
  }

  // Helper function to project a point in 3D to the virtual image coordinates
  cv::Point ProjectPoint(Eigen::Vector3d point3d);

  // Draw a line connecting two 3D points
  void DrawLine(Eigen::Vector3d start, Eigen::Vector3d end);

  // Draw coordinate frame for a target frame relative to the world frame
  // Axes are drawn (x,y,z) -> (red, green, blue)
  void DrawFrameAxes(Eigen::Affine3d H_world_target, std::string label = "",
                     double axis_scale = 0.25);

  // TODO<Jim>: Need to implement this, and maybe DrawRobotOutline
  void DrawBoardOutline(Eigen::Affine3d H_world_board, std::string label = "");

  Eigen::Affine3d H_world_viewpoint_;  // Where to view the world from
  cv::Mat image_;                      // Image to draw on
  cv::Mat camera_mat_;   // Virtual camera intrinsics (defines fov, center)
  cv::Mat dist_coeffs_;  // Distortion coefficients, if desired (only used in
                         // DrawFrameAxes
};
}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_VISUALIZE_ROBOT_H_
