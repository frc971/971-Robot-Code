#ifndef Y2022_VISION_POSE_ESTIMATOR_H_
#define Y2022_VISION_POSE_ESTIMATOR_H_

#include "opencv2/imgproc.hpp"
#include "y2022/vision/target_estimate_generated.h"

namespace y2022::vision {

class TargetEstimator {
 public:
  // Computes the location of the target.
  // blob_point is the mean (x, y) of blob pixels.
  // Adds angle_to_target and distance to the given builder.
  static void EstimateTargetLocation(cv::Point2i centroid,
                                     const cv::Mat &intrinsics,
                                     const cv::Mat &extrinsics,
                                     TargetEstimate::Builder *builder);

 private:
  // Height of the center of the tape (m)
  static constexpr double kTapeHeight = 2.58 + (0.05 / 2);
  // Horizontal distance from tape to center of hub (m)
  static constexpr double kUpperHubRadius = 1.22 / 2;
};

}  // namespace y2022::vision

#endif  // Y2022_VISION_POSE_ESTIMATOR_H_
