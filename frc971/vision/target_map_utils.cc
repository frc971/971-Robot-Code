#include "frc971/vision/target_map_utils.h"

#include <numbers>

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

template <typename Scalar>
Scalar degToRad(Scalar degrees) {
  return degrees * std::numbers::pi / 180.0;
}

Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc971::vision::TargetPoseFieldFbs *pose) {
  const frc971::vision::PositionField *position = pose->position();
  const frc971::vision::EulerAngle *euler_angle = pose->orientation();
  return (Eigen::Translation3d(
              Eigen::Vector3d(position->x(), position->y(), position->z())) *
          Eigen::Quaterniond(
              Eigen::AngleAxisd(degToRad(euler_angle->x() - 90.0),
                                Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(degToRad(euler_angle->y()),
                                Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(degToRad(euler_angle->z()),
                                Eigen::Vector3d::UnitZ()))
              .normalized())
      .matrix();
}
}  // namespace frc971::vision
