#include "y2023/vision/vision_util.h"

#include "glog/logging.h"

namespace y2023::vision {

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2023::Constants &calibration_data, std::string_view node_name) {
  CHECK(calibration_data.has_cameras());
  for (const y2023::CameraConfiguration *candidate :
       *calibration_data.cameras()) {
    CHECK(candidate->has_calibration());
    if (candidate->calibration()->node_name()->string_view() != node_name) {
      continue;
    }
    return candidate->calibration();
  }
  LOG(FATAL) << ": Failed to find camera calibration for " << node_name;
}

cv::Mat CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  CHECK(!camera_calibration->has_turret_extrinsics())
      << "No turret on 2023 robot";

  cv::Mat result(4, 4, CV_32F,
                 const_cast<void *>(static_cast<const void *>(
                     camera_calibration->fixed_extrinsics()->data()->data())));
  result.convertTo(result, CV_64F);
  CHECK_EQ(result.total(),
           camera_calibration->fixed_extrinsics()->data()->size());

  return result;
}

cv::Mat CameraIntrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  cv::Mat result(3, 3, CV_32F,
                 const_cast<void *>(static_cast<const void *>(
                     camera_calibration->intrinsics()->data())));
  result.convertTo(result, CV_64F);
  CHECK_EQ(result.total(), camera_calibration->intrinsics()->size());

  return result;
}

cv::Mat CameraDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  const cv::Mat result(5, 1, CV_32F,
                       const_cast<void *>(static_cast<const void *>(
                           camera_calibration->dist_coeffs()->data())));
  CHECK_EQ(result.total(), camera_calibration->dist_coeffs()->size());
  return result;
}

}  // namespace y2023::vision
