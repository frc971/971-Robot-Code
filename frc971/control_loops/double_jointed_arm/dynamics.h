#ifndef FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DYNAMICS_H_
#define FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DYNAMICS_H_

#include "Eigen/Dense"
#include "frc971/control_loops/runge_kutta.h"
#include "gflags/gflags.h"

DECLARE_bool(gravity);

namespace frc971 {
namespace control_loops {
namespace arm {

struct ArmConstants {
  // Below, 0 refers to the proximal joint, and 1 refers to the distal joint.
  // Length of the joints in meters.
  double l0;
  double l1;

  // Mass of the joints in kilograms.
  double m0;
  double m1;

  // Moment of inertia of the joints in kg m^2
  double j0;
  double j1;

  // Radius of the center of mass of the joints in meters.
  double r0;
  double r1;

  // Gear ratios for the two joints.
  double g0;
  double g1;

  // motor constants.
  double efficiency_tweak;
  double stall_torque;
  double free_speed;
  double stall_current;
  double resistance;
  double Kv;
  double Kt;

  // Number of motors on the distal joint.
  double num_distal_motors;
};

// This class captures the dynamics of our system.  It doesn't actually need to
// store state yet, so everything can be constexpr and/or static.
//
// 0, 0 is straight up.
class Dynamics {
 public:
  Dynamics(ArmConstants arm_constants);
  // Generates K1-2 for the arm ODE.
  // K1 * d^2 theta / dt^2 + K2 * d theta / dt = K3 * V - K4 * d theta/dt
  // These matricies are missing the velocity factor for K2[1, 0], and K2[0, 1].
  // You probbaly want MatriciesForState.
  void NormilizedMatriciesForState(
      const ::Eigen::Matrix<double, 4, 1> &X,
      ::Eigen::Matrix<double, 2, 2> *K1_result,
      ::Eigen::Matrix<double, 2, 2> *K2_result) const {
    const double angle = X(0, 0) - X(2, 0);
    const double s = ::std::sin(angle);
    const double c = ::std::cos(angle);
    *K1_result << alpha_, c * beta_, c * beta_, gamma_;
    *K2_result << 0.0, s * beta_, -s * beta_, 0.0;
  }

  // Generates K1-2 for the arm ODE.
  // K1 * d^2 theta / dt^2 + K2 * d theta / dt = K3 * V - K4 * d theta/dt
  void MatriciesForState(const ::Eigen::Matrix<double, 4, 1> &X,
                         ::Eigen::Matrix<double, 2, 2> *K1_result,
                         ::Eigen::Matrix<double, 2, 2> *K2_result) const {
    NormilizedMatriciesForState(X, K1_result, K2_result);
    (*K2_result)(1, 0) *= X(1, 0);
    (*K2_result)(0, 1) *= X(3, 0);
  }

  // Calculates the joint torques as a function of the state and command.
  const ::Eigen::Matrix<double, 2, 1> TorqueFromCommand(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U) {
    const ::Eigen::Matrix<double, 2, 1> velocity =
        (::Eigen::Matrix<double, 2, 1>() << X(1, 0), X(3, 0)).finished();

    return K3_ * U - K4_ * velocity;
  }

  const ::Eigen::Matrix<double, 2, 1> CurrentFromTorque(
      const ::Eigen::Matrix<double, 2, 1> &torque) {
    return ::Eigen::DiagonalMatrix<double, 2>(
               1.0 / (arm_constants_.Kt * arm_constants_.g0),
               1.0 / (arm_constants_.Kt * arm_constants_.g1 *
                      arm_constants_.num_distal_motors)) *
           torque;
  }

  const ::Eigen::Matrix<double, 2, 1> CurrentFromCommand(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U) {
    return CurrentFromTorque(TorqueFromCommand(X, U));
  }

  // Computes the two joint torques given the state and the external force in
  // x, y.
  const ::Eigen::Matrix<double, 2, 1> TorqueFromForce(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &F) {
    const ::Eigen::Matrix<double, 2, 1> L0(std::sin(X(0)) * arm_constants_.l0,
                                           std::cos(X(0)) * arm_constants_.l0);
    const ::Eigen::Matrix<double, 2, 1> L1(std::sin(X(2)) * arm_constants_.l1,
                                           std::cos(X(2)) * arm_constants_.l1);

    const Eigen::Matrix<double, 2, 1> Fn1 =
        F - L0.normalized().dot(F) * L0.normalized();

    const double torque1 = L0.x() * Fn1.y() - L0.y() * Fn1.x();
    const double torque2 = L1.x() * F.y() - L1.y() * F.x();

    return ::Eigen::Matrix<double, 2, 1>(torque1, torque2);
  }

  // TODO(austin): We may want a way to provide K1 and K2 to save CPU cycles.

  // Calculates the acceleration given the current state and control input.
  const ::Eigen::Matrix<double, 4, 1> Acceleration(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U) const {
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    MatriciesForState(X, &K1, &K2);

    const ::Eigen::Matrix<double, 2, 1> velocity =
        (::Eigen::Matrix<double, 2, 1>() << X(1, 0), X(3, 0)).finished();

    const ::Eigen::Matrix<double, 2, 1> torque = K3_ * U - K4_ * velocity;
    const ::Eigen::Matrix<double, 2, 1> gravity_torque = GravityTorque(X);

    const ::Eigen::Matrix<double, 2, 1> accel =
        K1.inverse() * (torque + gravity_torque - K2 * velocity);

    return (::Eigen::Matrix<double, 4, 1>() << X(1, 0), accel(0, 0), X(3, 0),
            accel(1, 0))
        .finished();
  }

  // Calculates the acceleration given the current augmented kalman filter state
  // and control input.
  const ::Eigen::Matrix<double, 6, 1> EKFAcceleration(
      const ::Eigen::Matrix<double, 6, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U) const {
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    MatriciesForState(X.block<4, 1>(0, 0), &K1, &K2);

    const ::Eigen::Matrix<double, 2, 1> velocity =
        (::Eigen::Matrix<double, 2, 1>() << X(1, 0), X(3, 0)).finished();

    const ::Eigen::Matrix<double, 2, 1> torque =
        K3_ *
            (U +
             (::Eigen::Matrix<double, 2, 1>() << X(4, 0), X(5, 0)).finished()) -
        K4_ * velocity;
    const ::Eigen::Matrix<double, 2, 1> gravity_torque =
        GravityTorque(X.block<4, 1>(0, 0));

    const ::Eigen::Matrix<double, 2, 1> accel =
        K1.inverse() * (torque + gravity_torque - K2 * velocity);

    return (::Eigen::Matrix<double, 6, 1>() << X(1, 0), accel(0, 0), X(3, 0),
            accel(1, 0), 0.0, 0.0)
        .finished();
  }

  // Calculates the voltage required to follow the trajectory.  This requires
  // knowing the current state, desired angular velocity and acceleration.
  const ::Eigen::Matrix<double, 2, 1> FF_U(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &omega_t,
      const ::Eigen::Matrix<double, 2, 1> &alpha_t) const {
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    MatriciesForState(X, &K1, &K2);

    const ::Eigen::Matrix<double, 2, 1> gravity_torque = GravityTorque(X);

    return K3_inverse_ *
           (K1 * alpha_t + K2 * omega_t + K4_ * omega_t - gravity_torque);
  }

  const ::Eigen::Matrix<double, 2, 1> GravityTorque(
      const ::Eigen::Matrix<double, 4, 1> &X) const {
    const double accel_due_to_gravity = 9.8 * arm_constants_.efficiency_tweak;
    return (::Eigen::Matrix<double, 2, 1>()
                << (arm_constants_.r0 * arm_constants_.m0 +
                    arm_constants_.l0 * arm_constants_.m1) *
                       ::std::sin(X(0)) * accel_due_to_gravity,
            arm_constants_.r1 * arm_constants_.m1 * ::std::sin(X(2)) *
                accel_due_to_gravity)
               .finished() *
           (FLAGS_gravity ? 1.0 : 0.0);
  }

  const ::Eigen::Matrix<double, 4, 1> UnboundedDiscreteDynamics(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U, double dt) const {
    return ::frc971::control_loops::RungeKuttaU(
        [this](const auto &X, const auto &U) { return Acceleration(X, U); }, X,
        U, dt);
  }

  const ::Eigen::Matrix<double, 6, 1> UnboundedEKFDiscreteDynamics(
      const ::Eigen::Matrix<double, 6, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U, double dt) const {
    return ::frc971::control_loops::RungeKuttaU(
        [this](const auto &X, const auto &U) { return EKFAcceleration(X, U); },
        X, U, dt);
  }

  const ArmConstants arm_constants_;

  // K3, K4 matricies described above.
  const ::Eigen::Matrix<double, 2, 2> K3_;
  const ::Eigen::Matrix<double, 2, 2> K3_inverse_;
  const ::Eigen::Matrix<double, 2, 2> K4_;

 private:
  const double alpha_;
  const double beta_;
  const double gamma_;
};

}  // namespace arm
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DYNAMICS_H_
