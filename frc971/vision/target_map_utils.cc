#include "frc971/vision/target_map_utils.h"

namespace frc971::vision {
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc971::vision::TargetPoseFbs *pose) {
  const frc971::vision::Position *position = pose->position();
  const frc971::vision::Quaternion *quaternion = pose->orientation();
  return (Eigen::Translation3d(
              Eigen::Vector3d(position->x(), position->y(), position->z())) *
          Eigen::Quaterniond(quaternion->w(), quaternion->x(), quaternion->y(),
                             quaternion->z())
              .normalized())
      .matrix();
}
}  // namespace frc971::vision
