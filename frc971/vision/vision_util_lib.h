#ifndef FRC971_VISION_VISION_UTIL_LIB_H_
#define FRC971_VISION_VISION_UTIL_LIB_H_
#include <optional>
#include <string_view>

#include "opencv2/imgproc.hpp"

#include "frc971/vision/calibration_generated.h"

// Extract the CameraExtrinsics from a CameraCalibration struct
namespace frc971::vision {
std::optional<cv::Mat> CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

// Extract the CameraIntrinsics from a CameraCalibration struct
cv::Mat CameraIntrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

// Extract the CameraDistCoeffs from a CameraCalibration struct
cv::Mat CameraDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

// Get the camera number from a camera channel name, e.g., return 2 from
// "/camera2".  Returns nullopt if string doesn't start with "/camera" or does
// not have a number
std::optional<uint16_t> CameraNumberFromChannel(std::string camera_channel);

// Return a calibration filename to save to based on the given data
std::string CalibrationFilename(std::string calibration_folder,
                                std::string node_name, int team_number,
                                int camera_number, std::string camera_id,
                                std::string timestamp);

}  // namespace frc971::vision

#endif  // FRC971_VISION_VISION_UTIL_LIB_H_
