#ifndef Y2022_VISION_TARGET_ESTIMATOR_H_
#define Y2022_VISION_TARGET_ESTIMATOR_H_

#include <optional>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"
#include "y2022/vision/target_estimate_generated.h"

namespace y2022::vision {
// Class to estimate the polar coordinates and rotation from the camera to the
// target.
class TargetEstimator {
 public:
  TargetEstimator(cv::Mat intrinsics, cv::Mat extrinsics);

  // Runs the solver to estimate the target
  // centroids must be in sorted order from left to right on the circle.
  // TODO(milind): seems safer to sort them here.
  // If image != std::nullopt, the solver's progress will be displayed
  // graphically.
  void Solve(const std::vector<cv::Point> &centroids,
             std::optional<cv::Mat> image);

  // Cost function for the solver.
  // Takes in the rotation of the camera in the hub's frame, the horizontal
  // polar coordinates of the camera in the hub's frame, and the height of the
  // camera (can change if the robot is shaking).
  // Hub frame is relative to the center of the bottom of the hub.
  // Compares the projected pieces of tape with these values to the detected
  // blobs for calculating the cost.
  template <typename S>
  bool operator()(const S *const roll, const S *const pitch, const S *const yaw,
                  const S *const distance, const S *const theta,
                  const S *const camera_height, S *residual) const;

  inline double roll() const { return roll_; }
  inline double pitch() const { return pitch_; }
  inline double yaw() const { return yaw_; }

  inline double distance() const { return distance_; }
  inline double angle_to_camera() const { return angle_to_camera_; }
  inline double angle_to_target() const { return M_PI - yaw_; }
  inline double camera_height() const { return camera_height_; }

  // Draws the distance, angle, and rotation on the given image
  static void DrawEstimate(const TargetEstimate &target_estimate,
                           cv::Mat view_image);
  void DrawEstimate(cv::Mat view_image) const;

 private:
  // Height of the center of the tape (m)
  static constexpr double kTapeHeight = 2.58 + (0.05 / 2);
  // Horizontal distance from tape to center of hub (m)
  static constexpr double kUpperHubRadius = 1.22 / 2;
  static constexpr size_t kNumPiecesOfTape = 16;

  // 3d points of the visible pieces of tape in the hub's frame
  static const std::vector<cv::Point3d> kTapePoints;
  static std::vector<cv::Point3d> ComputeTapePoints();

  template <typename S>
  cv::Point_<S> DistanceFromTape(
      size_t centroid_index,
      const std::vector<cv::Point_<S>> &tape_points) const;

  std::vector<cv::Point> centroids_;
  std::optional<cv::Mat> image_;

  Eigen::Matrix3d intrinsics_;
  Eigen::Matrix4d extrinsics_;

  double roll_;
  double pitch_;
  double yaw_;

  double distance_;
  double angle_to_camera_;
  double camera_height_;
};

}  // namespace y2022::vision

#endif  // Y2022_VISION_TARGET_ESTIMATOR_H_
