#include "frc971/vision/vision_util_lib.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"

namespace frc971::vision {

std::optional<cv::Mat> CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  CHECK(!camera_calibration->has_turret_extrinsics())
      << "Turret not currently supported";

  if (!camera_calibration->has_fixed_extrinsics()) {
    return std::nullopt;
  }
  CHECK(camera_calibration->fixed_extrinsics()->has_data());
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

std::optional<uint16_t> CameraNumberFromChannel(std::string camera_channel) {
  if (camera_channel.find("/camera") == std::string::npos) {
    return std::nullopt;
  }
  // If the string doesn't end in /camera#, return nullopt
  uint16_t cam_len = std::string("/camera").length();
  if (camera_channel.length() != camera_channel.find("/camera") + cam_len + 1) {
    return std::nullopt;
  }

  uint16_t camera_number = std::stoi(
      camera_channel.substr(camera_channel.find("/camera") + cam_len, 1));
  return camera_number;
}

std::string CalibrationFilename(std::string calibration_folder,
                                std::string node_name, int team_number,
                                int camera_number, std::string camera_id,
                                std::string timestamp) {
  // Get rid of any fractional seconds-- we shouldn't need those and it makes
  // the string unnecessarily longer
  timestamp = timestamp.substr(0, timestamp.find("."));
  std::string calibration_filename =
      calibration_folder +
      absl::StrFormat("/calibration_%s-%d-%d_cam-%s_%s.json", node_name.c_str(),
                      team_number, camera_number, camera_id.c_str(),
                      timestamp.c_str());
  return calibration_filename;
}

}  // namespace frc971::vision
