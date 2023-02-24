#ifndef Y2023_LOCALIZER_UTILS_H_
#define Y2023_LOCALIZER_UTILS_H_

#include <Eigen/Dense>

#include "frc971/vision/target_map_generated.h"

namespace y2023::localizer {
// Converts a TargetPoseFbs into a transformation matrix.
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc971::vision::TargetPoseFbs *pose);
}  // namespace y2023::localizer

#endif  // Y2023_LOCALIZER_UTILS_H_
