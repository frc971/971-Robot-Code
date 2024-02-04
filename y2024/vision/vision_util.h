#ifndef Y2024_VISION_VISION_UTIL_H_
#define Y2024_VISION_VISION_UTIL_H_
#include <string_view>

#include "opencv2/imgproc.hpp"

#include "y2024/constants/constants_generated.h"

namespace y2024::vision {

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2024::Constants &calibration_data, std::string_view node_name);

}  // namespace y2024::vision

#endif  // Y2024_VISION_VISION_UTIL_H_
