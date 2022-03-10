#ifndef FRC971_VISION_EXTRINSICS_CALIBRATION_H_
#define FRC971_VISION_EXTRINSICS_CALIBRATION_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "frc971/vision/calibration_accumulator.h"

namespace frc971 {
namespace vision {

struct CalibrationParameters {
  Eigen::Quaternion<double> initial_orientation =
      Eigen::Quaternion<double>::Identity();
  Eigen::Quaternion<double> pivot_to_camera =
      Eigen::Quaternion<double>::Identity();
  Eigen::Quaternion<double> pivot_to_imu =
      Eigen::Quaternion<double>::Identity();
  Eigen::Quaternion<double> board_to_world =
      Eigen::Quaternion<double>::Identity();

  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> initial_state =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 3, 1> pivot_to_camera_translation =
      Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> pivot_to_imu_translation =
      Eigen::Matrix<double, 3, 1>::Zero();

  double gravity_scalar = 1.0;
  Eigen::Matrix<double, 3, 1> accelerometer_bias =
      Eigen::Matrix<double, 3, 1>::Zero();

  bool has_pivot = false;
};

// Solves the mounting problem given the calibration data and parameters.  The
// parameters are used as the seed to the solver.
void Solve(const CalibrationData &data,
           CalibrationParameters *calibration_parameters);

// Plots the calibrated results to help visualize the fit.
void Plot(const CalibrationData &data,
          const CalibrationParameters &calibration_parameters);

}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_EXTRINSICS_CALIBRATION_H_
