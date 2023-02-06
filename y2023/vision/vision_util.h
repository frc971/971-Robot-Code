#ifndef Y2023_VISION_VISION_UTIL_H_
#define Y2023_VISION_VISION_UTIL_H_
#include <string_view>

#include "y2023/constants/constants_generated.h"
namespace y2023::vision {

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2023::Constants &calibration_data, std::string_view node_name);
}
#endif  // Y2023_VISION_VISION_UTIL_H_
