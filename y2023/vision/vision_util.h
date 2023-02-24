#ifndef Y2023_VISION_VISION_UTIL_H_
#define Y2023_VISION_VISION_UTIL_H_
#include <string_view>

#include "opencv2/imgproc.hpp"
#include "y2023/constants/constants_generated.h"

namespace y2023::vision {

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2023::Constants &calibration_data, std::string_view node_name);

std::optional<cv::Mat> CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

cv::Mat CameraIntrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

cv::Mat CameraDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

}  // namespace y2023::vision

#endif  // Y2023_VISION_VISION_UTIL_H_
