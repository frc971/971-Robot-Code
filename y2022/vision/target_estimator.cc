#include "y2022/vision/target_estimator.h"

namespace y2022::vision {

TargetEstimateT TargetEstimator::EstimateTargetLocation(
    cv::Point2i blob_point, const cv::Mat &intrinsics,
    const cv::Mat &extrinsics) {
  const cv::Point2d focal_length(intrinsics.at<double>(0, 0),
                                 intrinsics.at<double>(1, 1));
  const cv::Point2d offset(intrinsics.at<double>(0, 2),
                           intrinsics.at<double>(1, 2));

  // Blob pitch in camera reference frame
  const double blob_pitch =
      std::atan(static_cast<double>(-(blob_point.y - offset.y)) /
                static_cast<double>(focal_length.y));
  const double camera_height = extrinsics.at<double>(2, 3);
  // Depth from camera to blob
  const double depth = (kTapeHeight - camera_height) / std::tan(blob_pitch);

  TargetEstimateT target;
  target.angle_to_target =
      std::atan2(static_cast<double>(blob_point.x - offset.x),
                 static_cast<double>(focal_length.x));
  target.distance =
      (depth / std::cos(target.angle_to_target)) + kUpperHubRadius;

  return target;
}

}  // namespace y2022::vision
