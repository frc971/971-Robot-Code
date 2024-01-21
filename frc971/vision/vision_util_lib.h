#ifndef FRC971_VISION_VISION_UTIL_LIB_H_
#define FRC971_VISION_VISION_UTIL_LIB_H_
#include <optional>
#include <string_view>

#include "opencv2/imgproc.hpp"

#include "frc971/vision/calibration_generated.h"

namespace frc971::vision {
std::optional<cv::Mat> CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

cv::Mat CameraIntrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

cv::Mat CameraDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

}  // namespace frc971::vision

#endif  // FRC971_VISION_VISION_UTIL_LIB_H_
