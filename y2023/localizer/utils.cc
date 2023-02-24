#include "y2023/localizer/utils.h"

namespace y2023::localizer {
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc971::vision::TargetPoseFbs *pose) {
  const frc971::vision::Position *position = pose->position();
  const frc971::vision::Quaternion *quaternion = pose->orientation();
  return (Eigen::Translation3d(
              Eigen::Vector3d(position->x(), position->y(), position->z())) *
          Eigen::Quaterniond(quaternion->w(), quaternion->x(), quaternion->y(),
                             quaternion->z()))
      .matrix();
}
}  // namespace y2023::localizer
