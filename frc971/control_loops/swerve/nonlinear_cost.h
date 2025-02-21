#pragma once

#include "absl/log/check.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "frc971/control_loops/swerve/auto_diff_jacobian.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "simplified_dynamics.h"

namespace frc971::control_loops::swerve {
template <int NStates, typename Scalar = double>
class NonLinearCost {
 public:
  typedef Eigen::Matrix<Scalar, NStates, NStates> StateSquare;
  typedef Eigen::Matrix<Scalar, NStates, 1> State;
  typedef SimplifiedDynamics<Scalar> SimpleDynamics;

  NonLinearCost(const Scalar Q_slip, SimpleDynamics::Parameters params)
      : Q_slip_(Q_slip), params_(params) {}

  template <typename LocalScalar>
  Eigen::Matrix<LocalScalar, NStates, 1> operator()(
      const Eigen::Map<const Eigen::Matrix<LocalScalar, NStates, 1>> X) const {
    typedef Eigen::Matrix<LocalScalar, 3, 1> Vector3;
    typedef Eigen::Matrix<LocalScalar, 2, 1> Vector2;
    Eigen::Matrix<LocalScalar, NStates, 1> cost =
        Eigen::Matrix<LocalScalar, NStates, 1>::Zero();
    Eigen::Rotation2D<LocalScalar> rot(X[SimpleDynamics::States::kTheta]);

    for (size_t module = 0; module < params_.modules.size(); module++) {
      Vector2 rthetad_2 = rot * params_.modules[module].position;
      Vector3 rthetad_3(rthetad_2[0], rthetad_2[1],
                        static_cast<LocalScalar>(0.0));

      Vector3 v_mod_omega =
          Vector3(static_cast<LocalScalar>(0.0), static_cast<LocalScalar>(0.0),
                  X[SimpleDynamics::States::kOmega])
              .cross(rthetad_3);
      CHECK(v_mod_omega[2] == 0)
          << "r x omega should not have a z component. vector: " << v_mod_omega;
      Vector2 v_mod_omega_2(v_mod_omega[0], v_mod_omega[1]);

      Vector2 v_mod =
          v_mod_omega_2 + X.segment(SimpleDynamics::States::kVx,
                                    SimpleDynamics::States::kVy + 1);

      Vector2 v_mod_wheel =
          Eigen::Rotation2D(-X[SimpleDynamics::States::kTheta] -
                            X[SimpleDynamics::States::kThetas0 +
                              SimpleDynamics::States::kLength * module]) *
          v_mod;

      cost[SimpleDynamics::States::kOmegad0 +
           SimpleDynamics::States::kLength * module] =
          -Q_slip_ * slip(X[SimpleDynamics::States::kOmegad0 +
                            SimpleDynamics::States::kLength * module],
                          v_mod_wheel[0], module);
    }

    return cost;
  }

  template <typename LocalScalar>
  Eigen::Matrix<LocalScalar, NStates, 1> operator()(
      const Eigen::Matrix<LocalScalar, NStates, 1> &X) const {
    return (*this)(
        Eigen::Map<const Eigen::Matrix<LocalScalar, NStates, 1>>(X.data()));
  }

  StateSquare LinearizeCost(const State &X) {
    State parameters = X;
    const StateSquare jacobian =
        AutoDiffJacobian<Scalar, NonLinearCost, NStates, NStates>::Jacobian(
            *this, parameters);
    return jacobian;
  }

  template <typename LocalScalar>
  LocalScalar slip(LocalScalar omegad, LocalScalar v_long,
                   size_t module) const {
    LocalScalar top = omegad * params_.modules[module].wheel_radius - v_long;
    return top * top /
           std::max(v_long * v_long, static_cast<LocalScalar>(0.01));
  }

 private:
  Scalar Q_slip_;
  SimpleDynamics::Parameters params_;
};
}  // namespace frc971::control_loops::swerve
