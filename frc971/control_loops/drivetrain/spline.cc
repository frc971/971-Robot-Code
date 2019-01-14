#include "frc971/control_loops/drivetrain/spline.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

::Eigen::Matrix<double, 2, 6> Spline4To6(
    const ::Eigen::Matrix<double, 2, 4> &control_points) {
  ::Eigen::Matrix<double, 2, 6> new_control_points;
  // a' = a
  // b' = (2a + 3b) / 5
  // c' = (a + 6b + 3c) / 10
  // d' = (d + 6c + 3b) / 10
  // e' = (2d + 3c) / 5
  // f' = d
  new_control_points.block<2, 1>(0, 0) = control_points.block<2, 1>(0, 0);
  new_control_points.block<2, 1>(0, 1) =
      (2.0 * control_points.block<2, 1>(0, 0) +
       3.0 * control_points.block<2, 1>(0, 1)) /
      5.0;
  new_control_points.block<2, 1>(0, 2) =
      (control_points.block<2, 1>(0, 0) +
       6.0 * control_points.block<2, 1>(0, 1) +
       3.0 * control_points.block<2, 1>(0, 2)) /
      10.0;
  new_control_points.block<2, 1>(0, 3) =
      (control_points.block<2, 1>(0, 3) +
       6.0 * control_points.block<2, 1>(0, 2) +
       3.0 * control_points.block<2, 1>(0, 1)) /
      10.0;
  new_control_points.block<2, 1>(0, 4) =
      (2.0 * control_points.block<2, 1>(0, 3) +
       3.0 * control_points.block<2, 1>(0, 2)) /
      5.0;
  new_control_points.block<2, 1>(0, 5) = control_points.block<2, 1>(0, 3);
  return new_control_points;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
