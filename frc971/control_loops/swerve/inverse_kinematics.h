#ifndef FRC971_CONTROL_LOOPS_SWERVE_INVERSE_KINEMATICS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_INVERSE_KINEMATICS_H_
#include "aos/util/math.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
namespace frc971::control_loops::swerve {
// Class to do straightforwards inverse kinematics of a swerve drivebase. This
// is meant largely as a sanity-check/initializer for more sophisticated
// methods. This calculates which directions the modules must be pointed to
// cause them to be pointed directly along the direction of motion of the
// drivebase. Accounting for slip angles and the such must be done as part of
// more sophisticated inverse dynamics.
template <typename Scalar>
class InverseKinematics {
 public:
  using ModuleParams = SimplifiedDynamics<Scalar>::ModuleParams;
  using Parameters = SimplifiedDynamics<Scalar>::Parameters;
  using States = SimplifiedDynamics<Scalar>::States;
  using State = SimplifiedDynamics<Scalar>::template VelocityState<Scalar>;
  InverseKinematics(const Parameters &params) : params_(params) {}

  // Uses kVx, kVy, kTheta, and kOmega from the input goal state for the
  // absolute kinematics. Also uses the specified theta values to bias theta
  // output values towards the current state (i.e., if the module 0 theta is
  // currently 0 and we are asked to drive straight backwards, this will prefer
  // a theta of zero rather than a theta of pi).
  State Solve(const State &goal) {
    State result = goal;
    for (size_t module_index = 0; module_index < params_.modules.size();
         ++module_index) {
      SolveModule(goal, params_.modules[module_index].position,
                  &result(States::kThetas0 + 3 * module_index),
                  &result(States::kOmegas0 + 3 * module_index));
    }
    return result;
  }

  void SolveModule(const State &goal,
                   const Eigen::Matrix<Scalar, 2, 1> &module_position,
                   Scalar *module_theta, Scalar *module_omega) {
    const Scalar vx = goal(States::kVx);
    const Scalar vy = goal(States::kVy);
    const Scalar omega = goal(States::kOmega);
    // module_velocity_in_robot_frame = R(-theta) * robot_vel +
    //     omega.cross(module_position);
    // module_vel_x = (cos(-theta) * vx - sin(-theta) * vy) - omega * module_y
    // module_vel_y = (sin(-theta) * vx + cos(-theta) * vy) + omega * module_x
    // module_theta = atan2(module_vel_y, module_vel_x)
    // module_omega = datan2(module_vel_y, module_vel_x) / dt
    // datan2(y, x) / dt = (x * dy/dt - y * dx / dt) / (x^2 + y^2)
    // robot accelerations are assumed to be zero.
    // dmodule_vel_x / dt = (sin(-theta) * vx + cos(-theta) * vy) * omega
    // dmodule_vel_y / dt = (-cos(-theta) * vx + sin(-theta) * vy) * omega
    const Scalar ctheta = std::cos(-goal(States::kTheta));
    const Scalar stheta = std::sin(-goal(States::kTheta));
    const Scalar module_vel_x =
        (ctheta * vx - stheta * vy) - omega * module_position.y();
    const Scalar module_vel_y =
        (stheta * vx + ctheta * vy) + omega * module_position.x();
    const Scalar nominal_module_theta = atan2(module_vel_y, module_vel_x);
    // If the current module theta is more than 90 deg from the desired theta,
    // flip the desired theta by 180 deg.
    Scalar desired_theta;
    if (std::abs(aos::math::DiffAngle(nominal_module_theta, *module_theta)) >
        M_PI_2) {
      desired_theta = aos::math::NormalizeAngle(nominal_module_theta + M_PI);
    } else {
      desired_theta = nominal_module_theta;
    }
    *module_theta += aos::math::NormalizeAngle(desired_theta - *module_theta);
    const Scalar module_accel_x = (stheta * vx + ctheta * vy) * omega;
    const Scalar module_accel_y = (-ctheta * vx + stheta * vy) * omega;
    const Scalar module_vel_norm_squared =
        (module_vel_x * module_vel_x + module_vel_y * module_vel_y);
    if (module_vel_norm_squared < 1e-5) {
      // Prevent poor conditioning of module velocities at near-zero speeds.
      *module_omega = 0.0;
    } else {
      *module_omega =
          (module_vel_x * module_accel_y - module_vel_y * module_accel_x) /
          module_vel_norm_squared;
    }
  }

 private:
  Parameters params_;
};
}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_INVERSE_KINEMATICS_H_
