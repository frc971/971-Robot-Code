#ifndef FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_EKF_H_
#define FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_EKF_H_

#include "Eigen/Dense"

#include "frc971/control_loops/double_jointed_arm/dynamics.h"

namespace frc971 {
namespace control_loops {
namespace arm {

// An extended kalman filter for the Arm.
// Our states are:
//   [theta0, omega0, theta1, omega1, voltage error0, voltage error1]
class EKF {
 public:
  EKF(const Dynamics *dynamics);

  // Resets the internal state back to X.  Resets the torque disturbance to 0.
  void Reset(const ::Eigen::Matrix<double, 4, 1> &X);
  // TODO(austin): Offset the internal state when we calibrate.

  // Corrects the state estimate with the provided sensor reading.
  void Correct(const ::Eigen::Matrix<double, 2, 1> &Y, double dt);

  // Predicts the next state and covariance given the control input.
  void Predict(const ::Eigen::Matrix<double, 2, 1> &U, double dt);

  // Returns the current state and covariance estimates.
  const ::Eigen::Matrix<double, 6, 1> &X_hat() const { return X_hat_; }
  double X_hat(int i) const { return X_hat_(i); }
  const ::Eigen::Matrix<double, 6, 6> &P() const { return P_; }
  const ::Eigen::Matrix<double, 6, 6> &P_reset() const { return P_reset_; }
  const ::Eigen::Matrix<double, 6, 6> &P_half_converged() const {
    return P_half_converged_;
  }
  const ::Eigen::Matrix<double, 6, 6> &P_converged() const {
    return P_converged_;
  }

 private:
  ::Eigen::Matrix<double, 6, 1> X_hat_;
  ::Eigen::Matrix<double, 6, 6> P_;
  ::Eigen::Matrix<double, 6, 6> P_half_converged_;
  ::Eigen::Matrix<double, 6, 6> P_converged_;
  ::Eigen::Matrix<double, 6, 6> P_reset_;

  const Dynamics *dynamics_;
};

}  // namespace arm
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_EKF_H_
