#ifndef Y2022_VISION_TARGET_ESTIMATOR_H_
#define Y2022_VISION_TARGET_ESTIMATOR_H_

#include <optional>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"
#include "y2022/vision/blob_detector.h"
#include "y2022/vision/target_estimate_generated.h"

namespace y2022::vision {

// Class to estimate the distance and rotation of the camera from the
// target.
class TargetEstimator {
 public:
  TargetEstimator(cv::Mat intrinsics, cv::Mat extrinsics);

  // Runs the solver to estimate the target
  // If image != std::nullopt, the solver's progress will be displayed
  // graphically.
  void Solve(const std::vector<BlobDetector::BlobStats> &blob_stats,
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

  inline double confidence() const { return confidence_; }

  // Draws the distance, angle, rotation, and projected tape on the given image
  void DrawEstimate(cv::Mat view_image) const;

 private:
  // 3d points of the visible pieces of tape in the hub's frame
  static const std::vector<cv::Point3d> kTapePoints;
  // 3d outer points of the middle piece of tape in the hub's frame,
  // clockwise around the rectangle
  static const std::array<cv::Point3d, 4> kMiddleTapePiecePoints;

  // Computes matrix of hub in camera's frame
  template <typename S>
  Eigen::Transform<S, 3, Eigen::Affine> ComputeHubCameraTransform(
      S roll, S pitch, S yaw, S distance, S theta, S camera_height) const;

  template <typename S>
  cv::Point_<S> ProjectToImage(
      cv::Point3d tape_point_hub,
      const Eigen::Transform<S, 3, Eigen::Affine> &H_hub_camera) const;

  template <typename S>
  cv::Point_<S> DistanceFromTape(
      size_t centroid_index,
      const std::vector<cv::Point_<S>> &tape_points) const;

  void DrawProjectedHub(const std::vector<cv::Point2d> &tape_points_proj,
                        cv::Mat view_image) const;

  std::vector<BlobDetector::BlobStats> blob_stats_;
  size_t middle_blob_index_;
  std::optional<cv::Mat> image_;

  Eigen::Matrix3d intrinsics_;
  Eigen::Matrix4d extrinsics_;

  double roll_;
  double pitch_;
  double yaw_;

  double distance_;
  double angle_to_camera_;
  double camera_height_;
  double confidence_;
};

}  // namespace y2022::vision

#endif  // Y2022_VISION_TARGET_ESTIMATOR_H_
