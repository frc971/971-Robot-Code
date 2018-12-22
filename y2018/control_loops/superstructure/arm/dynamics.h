#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_DYNAMICS_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_DYNAMICS_H_

#include "Eigen/Dense"

#include "frc971/control_loops/runge_kutta.h"
#include "gflags/gflags.h"

DECLARE_bool(gravity);

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

// This class captures the dynamics of our system.  It doesn't actually need to
// store state yet, so everything can be constexpr and/or static.
class Dynamics {
 public:
  // Below, 1 refers to the proximal joint, and 2 refers to the distal joint.
  // Length of the joints in meters.
  static constexpr double kL1 = 46.25 * 0.0254;
  static constexpr double kL2 = 41.80 * 0.0254;

  // Mass of the joints in kilograms.
  static constexpr double kM1 = 9.34 / 2.2;
  static constexpr double kM2 = 9.77 / 2.2;

  // Moment of inertia of the joints in kg m^2
  static constexpr double kJ1 = 2957.05 * 0.0002932545454545454;
  static constexpr double kJ2 = 2824.70 * 0.0002932545454545454;

  // Radius of the center of mass of the joints in meters.
  static constexpr double r1 = 21.64 * 0.0254;
  static constexpr double r2 = 26.70 * 0.0254;

  // Gear ratios for the two joints.
  static constexpr double kG1 = 140.0;
  static constexpr double kG2 = 90.0;

  // MiniCIM motor constants.
  static constexpr double kEfficiencyTweak = 0.95;
  static constexpr double kStallTorque = 1.41 * kEfficiencyTweak;
  static constexpr double kFreeSpeed = (5840.0 / 60.0) * 2.0 * M_PI;
  static constexpr double kStallCurrent = 89.0;
  static constexpr double kResistance = 12.0 / kStallCurrent;
  static constexpr double Kv = kFreeSpeed / 12.0;
  static constexpr double Kt = kStallTorque / kStallCurrent;

  // Number of motors on the distal joint.
  static constexpr double kNumDistalMotors = 2.0;

  static constexpr double kAlpha = kJ1 + r1 * r1 * kM1 + kL1 * kL1 * kM2;
  static constexpr double kBeta = kL1 * r2 * kM2;
  static constexpr double kGamma = kJ2 + r2 * r2 * kM2;

  // K3, K4 matricies described below.
  static const ::Eigen::Matrix<double, 2, 2> K3;
  static const ::Eigen::Matrix<double, 2, 2> K3_inverse;
  static const ::Eigen::Matrix<double, 2, 2> K4;

  // Generates K1-2 for the arm ODE.
  // K1 * d^2 theta / dt^2 + K2 * d theta / dt = K3 * V - K4 * d theta/dt
  // These matricies are missing the velocity factor for K2[1, 0], and K2[0, 1].
  // You probbaly want MatriciesForState.
  static void NormilizedMatriciesForState(
      const ::Eigen::Matrix<double, 4, 1> &X,
      ::Eigen::Matrix<double, 2, 2> *K1_result,
      ::Eigen::Matrix<double, 2, 2> *K2_result) {
    const double angle = X(0, 0) - X(2, 0);
    const double s = ::std::sin(angle);
    const double c = ::std::cos(angle);
    *K1_result << kAlpha, c * kBeta, c * kBeta, kGamma;
    *K2_result << 0.0, s * kBeta, -s * kBeta, 0.0;
  }

  // Generates K1-2 for the arm ODE.
  // K1 * d^2 theta / dt^2 + K2 * d theta / dt = K3 * V - K4 * d theta/dt
  static void MatriciesForState(const ::Eigen::Matrix<double, 4, 1> &X,
                                ::Eigen::Matrix<double, 2, 2> *K1_result,
                                ::Eigen::Matrix<double, 2, 2> *K2_result) {
    NormilizedMatriciesForState(X, K1_result, K2_result);
    (*K2_result)(1, 0) *= X(1, 0);
    (*K2_result)(0, 1) *= X(3, 0);
  }

  // TODO(austin): We may want a way to provide K1 and K2 to save CPU cycles.

  // Calculates the acceleration given the current state and control input.
  static const ::Eigen::Matrix<double, 4, 1> Acceleration(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U) {
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    MatriciesForState(X, &K1, &K2);

    const ::Eigen::Matrix<double, 2, 1> velocity =
        (::Eigen::Matrix<double, 2, 1>() << X(1, 0), X(3, 0)).finished();

    const ::Eigen::Matrix<double, 2, 1> torque = K3 * U - K4 * velocity;
    const ::Eigen::Matrix<double, 2, 1> gravity_torque = GravityTorque(X);

    const ::Eigen::Matrix<double, 2, 1> accel =
        K1.inverse() * (torque + gravity_torque - K2 * velocity);

    return (::Eigen::Matrix<double, 4, 1>() << X(1, 0), accel(0, 0), X(3, 0),
            accel(1, 0))
        .finished();
  }

  // Calculates the acceleration given the current augmented kalman filter state
  // and control input.
  static const ::Eigen::Matrix<double, 6, 1> EKFAcceleration(
      const ::Eigen::Matrix<double, 6, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U) {
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    MatriciesForState(X.block<4, 1>(0, 0), &K1, &K2);

    const ::Eigen::Matrix<double, 2, 1> velocity =
        (::Eigen::Matrix<double, 2, 1>() << X(1, 0), X(3, 0)).finished();

    const ::Eigen::Matrix<double, 2, 1> torque =
        K3 *
            (U +
             (::Eigen::Matrix<double, 2, 1>() << X(4, 0), X(5, 0)).finished()) -
        K4 * velocity;
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
  static const ::Eigen::Matrix<double, 2, 1> FF_U(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &omega_t,
      const ::Eigen::Matrix<double, 2, 1> &alpha_t) {
    ::Eigen::Matrix<double, 2, 2> K1;
    ::Eigen::Matrix<double, 2, 2> K2;

    MatriciesForState(X, &K1, &K2);

    const ::Eigen::Matrix<double, 2, 1> gravity_torque = GravityTorque(X);

    return K3_inverse *
           (K1 * alpha_t + K2 * omega_t + K4 * omega_t - gravity_torque);
  }

  static ::Eigen::Matrix<double, 2, 1> GravityTorque(
      const ::Eigen::Matrix<double, 4, 1> &X) {
    constexpr double kAccelDueToGravity = 9.8 * kEfficiencyTweak;
    return (::Eigen::Matrix<double, 2, 1>() << (r1 * kM1 + kL1 * kM2) *
                                                   ::std::sin(X(0)) *
                                                   kAccelDueToGravity,
            r2 * kM2 * ::std::sin(X(2)) * kAccelDueToGravity)
               .finished() *
           (FLAGS_gravity ? 1.0 : 0.0);
  }

  static const ::Eigen::Matrix<double, 4, 1> UnboundedDiscreteDynamics(
      const ::Eigen::Matrix<double, 4, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U, double dt) {
    return ::frc971::control_loops::RungeKuttaU(Dynamics::Acceleration, X, U,
                                                dt);
  }

  static const ::Eigen::Matrix<double, 6, 1> UnboundedEKFDiscreteDynamics(
      const ::Eigen::Matrix<double, 6, 1> &X,
      const ::Eigen::Matrix<double, 2, 1> &U, double dt) {
    return ::frc971::control_loops::RungeKuttaU(Dynamics::EKFAcceleration, X, U,
                                                dt);
  }
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_DYNAMICS_H_
