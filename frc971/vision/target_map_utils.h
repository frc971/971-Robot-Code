#ifndef FRC971_VISION_TARGET_MAP_UTILS_H_
#define FRC971_VISION_TARGET_MAP_UTILS_H_

#include <Eigen/Dense>

#include "frc971/vision/target_map_field_generated.h"
#include "frc971/vision/target_map_generated.h"

namespace frc971::vision {
// Converts a TargetPoseFbs into a transformation matrix.
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc971::vision::TargetPoseFbs *pose);
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc971::vision::TargetPoseFieldFbs *pose);

}  // namespace frc971::vision

#endif  // FRC971_VISION_TARGET_MAP_UTILS_H_
