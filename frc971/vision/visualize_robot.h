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
  VisualizeRobot(cv::Size default_size = cv::Size(1280, 720))
      : default_size_(default_size) {}

  // Set image on which to draw
  void SetImage(cv::Mat image) { image_ = image; }

  // Sets image to all black.
  // Uses default_size_ if no image has been set yet, else image_.size()
  void ClearImage() {
    auto image_size = (image_.data == nullptr ? default_size_ : image_.size());
    cv::Mat black_image_mat = cv::Mat::zeros(image_size, CV_8UC3);
    SetImage(black_image_mat);
  }

  // Set the viewpoint of the camera relative to a global origin
  void SetViewpoint(Eigen::Affine3d view_origin) {
    H_world_viewpoint_ = view_origin;
  }

  // Set camera parameters (for projection into a virtual view)
  void SetCameraParameters(cv::Mat camera_intrinsics) {
    camera_mat_ = camera_intrinsics.clone();
  }

  // Set distortion coefficients (defaults to 0)
  void SetDistortionCoefficients(cv::Mat dist_coeffs) {
    dist_coeffs_ = dist_coeffs.clone();
  }

  // Sets up a default camera view 10 m above the origin, pointing down
  // Uses image_width and focal_length to define a default camera projection
  // matrix
  void SetDefaultViewpoint(int image_width = 1000,
                           double focal_length = 1000.0);

  // Helper function to project a point in 3D to the virtual image coordinates
  cv::Point ProjectPoint(Eigen::Vector3d point3d);

  // Draw a line connecting two 3D points
  void DrawLine(Eigen::Vector3d start, Eigen::Vector3d end,
                cv::Scalar color = cv::Scalar(0, 200, 0));

  // Draw coordinate frame for a target frame relative to the world frame
  // Axes are drawn (x,y,z) -> (red, green, blue)
  void DrawFrameAxes(Eigen::Affine3d H_world_target, std::string label = "",
                     cv::Scalar label_color = cv::Scalar(0, 0, 255),
                     double axis_scale = 0.25);

  // TODO<Jim>: Also implement DrawBoardOutline?  Maybe one function w/
  // parameters?
  void DrawRobotOutline(Eigen::Affine3d H_world_robot, std::string label = "",
                        cv::Scalar color = cv::Scalar(0, 200, 0));

  Eigen::Affine3d H_world_viewpoint_;  // Where to view the world from
  cv::Mat image_;                      // Image to draw on
  cv::Mat camera_mat_;     // Virtual camera intrinsics (defines fov, center)
  cv::Mat dist_coeffs_;    // Distortion coefficients, if desired (only used in
                           // DrawFrameAxes
  cv::Size default_size_;  // Default image size
};
}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_VISUALIZE_ROBOT_H_
