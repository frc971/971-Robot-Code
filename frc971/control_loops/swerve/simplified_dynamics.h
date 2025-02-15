#ifndef FRC971_CONTROL_LOOPS_SWERVE_SIMPLIFIED_DYNAMICS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_SIMPLIFIED_DYNAMICS_H_
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/util/math.h"
#include "frc971/control_loops/swerve/auto_diff_jacobian.h"
#include "frc971/control_loops/swerve/dynamics.h"
#include "frc971/control_loops/swerve/linearization_utils.h"
#include "frc971/control_loops/swerve/motors.h"

namespace frc971::control_loops::swerve {

// Provides a simplified set of physics representing a swerve drivetrain.
// Broadly speaking, these dynamics model:
// * Standard motors on the drive and steer axes, with no coupling between
//   the motors.
// * Assume that the steer direction of each module is only influenced by
//   the inertia of the motor rotor plus some small aligning force.
// * Assume that torque from the drive motor is transferred directly to the
//   carpet.
// * Assume that a lateral force on the wheel is generated proportional to the
//   slip angle of the wheel.
//
// This class is templated on a Scalar that is used to determine whether you
// want to use double or single-precision floats for the calculations here.
//
// Several individual methods on this class are also templated on a LocalScalar
// type. This is provided to allow those methods to be called with ceres Jets to
// do autodifferentiation within various solvers/jacobian calculators.
template <typename Scalar = double>
class SimplifiedDynamics {
 public:
  struct ModuleParams {
    // Module position relative to the center of mass of the robot.
    Eigen::Matrix<Scalar, 2, 1> position;
    // Coefficient dictating how much sideways force is generated at a given
    // slip angle. Units are effectively Newtons / radian of slip.
    Scalar slip_angle_coefficient;
    // Coefficient dicating how much the steer wheel is forced into alignment
    // by the motion of the wheel over the ground (i.e., we are assuming that
    // if you push the robot along it will cause the wheels to eventually
    // align with the direction of motion).
    // In radians / sec^2 / radians of slip angle.
    Scalar slip_angle_alignment_coefficient;
    // Parameters for the steer and drive motors.
    Motor steer_motor;
    Motor drive_motor;
    // radians of module steering = steer_ratio * radians of motor shaft
    Scalar steer_ratio;
    // meters of driving = drive_ratio * radians of motor shaft
    Scalar drive_ratio;
    // radius of wheel
    Scalar wheel_radius;
    // A number to add to the steer module inertia (beyond just the rotor
    // inertia of the motor). In practice this may end up just serving as a bit
    // of a drag term (or be unused entirely), as we don't currently believe the
    // *actual* inertia of the gearbox/wheel should be significant compared to
    // the rotor inertia.
    Scalar extra_steer_inertia;
  };
  struct Parameters {
    // Mass of the robot, in kg.
    Scalar mass;
    // Moment of inertia of the robot about the yaw axis, in kg * m^2.
    Scalar moment_of_inertia;
    // Note: While this technically would support an arbitrary number of
    // modules, the statically-sized state vectors do limit us to 4 modules
    // currently, and it should not be counted on that other pieces of code will
    // be able to support non-4-module swerves.
    std::vector<ModuleParams> modules;
    // How much to trust the accelerometer-based velocity versus the
    // encoder-based velocity for robot velocity estimation, from 0 (no
    // accelerometer) to 1 (only accelerometer).
    Scalar accel_weight;
  };
  enum States {
    // Thetas* and Omegas* are the yaw and yaw rate of the indicated modules.
    // (note that we do not actually need to track drive speed per module,
    // as with current control we have the ability to directly command torque
    // to those motors; however, if we wished to account for the saturation
    // limits on the motor, then we would need to have access to those states,
    // although they can be fully derived from the robot vx, vy, theta, and
    // omega).
    kThetas0 = 0,
    kOmegas0 = 1,
    kThetas1 = 2,
    kOmegas1 = 3,
    kThetas2 = 4,
    kOmegas2 = 5,
    kThetas3 = 6,
    kOmegas3 = 7,
    // Robot yaw, in radians.
    kTheta = 8,
    // Robot speed in the global frame, in meters / sec.
    kVx = 9,
    kVy = 10,
    // Robot yaw rate, in radians / sec.
    kOmega = 11,
    kNumVelocityStates = 12,
    // Augmented states for doing position control.
    // Robot X position in the global frame.
    kX = 12,
    // Robot Y position in the global frame.
    kY = 13,
    kNumPositionStates = 14,
  };
  using Inputs = InputStates::States;

  template <typename ScalarT = Scalar>
  using VelocityState = Eigen::Matrix<ScalarT, kNumVelocityStates, 1>;
  template <typename ScalarT = Scalar>
  using PositionState = Eigen::Matrix<ScalarT, kNumPositionStates, 1>;
  template <typename ScalarT = Scalar>
  using VelocityStateSquare =
      Eigen::Matrix<ScalarT, kNumVelocityStates, kNumVelocityStates>;
  template <typename ScalarT = Scalar>
  using PositionStateSquare =
      Eigen::Matrix<ScalarT, kNumPositionStates, kNumPositionStates>;
  template <typename ScalarT = Scalar>
  using PositionBMatrix =
      Eigen::Matrix<ScalarT, kNumPositionStates, kNumInputs>;
  template <typename ScalarT = Scalar>
  using VelocityBMatrix =
      Eigen::Matrix<ScalarT, kNumVelocityStates, kNumInputs>;
  template <typename ScalarT = Scalar>
  using Input = Eigen::Matrix<ScalarT, kNumInputs, 1>;

  // Provide an interface to the dynamics which overrides the virtual methods in
  // the DynamicsInterface, to enable use in controllers/filters that expect the
  // dynamics to present a particular interface.
  class VirtualVelocityDynamics
      : public DynamicsInterface<Scalar, kNumVelocityStates, kNumInputs> {
   public:
    using LinearDynamics = DynamicsInterface<Scalar, kNumVelocityStates,
                                             kNumInputs>::LinearDynamics;
    VirtualVelocityDynamics(const Parameters &params) : dynamics_(params) {}
    VelocityState<> operator()(const VelocityState<> &X,
                               const Input<> &U) const override {
      return dynamics_.VelocityDynamics(X, U);
    }
    LinearDynamics LinearizeDynamics(const VelocityState<> &X,
                                     const Input<> &U) const override {
      auto pair = dynamics_.LinearizedVelocityDynamics(X, U);
      return {pair.first, pair.second};
    }

   private:
    const SimplifiedDynamics<Scalar> dynamics_;
  };

  SimplifiedDynamics(const Parameters &params) : params_(params) {
    for (size_t module_index = 0; module_index < params_.modules.size();
         ++module_index) {
      module_dynamics_.emplace_back(params_, module_index);
    }
  }

  // Returns the derivative of state for the given state and input.
  template <typename LocalScalar>
  PositionState<LocalScalar> Dynamics(const PositionState<LocalScalar> &state,
                                      const Input<LocalScalar> &input) const {
    PositionState<LocalScalar> Xdot = PositionState<LocalScalar>::Zero();

    for (const ModuleDynamics &module : module_dynamics_) {
      Xdot += module.PartialDynamics(state, input);
    }

    // And finally catch the global states:
    Xdot(kX) = state(kVx);
    Xdot(kY) = state(kVy);
    Xdot(kTheta) = state(kOmega);

    return Xdot;
  }

  template <typename LocalScalar>
  VelocityState<LocalScalar> VelocityDynamics(
      const VelocityState<LocalScalar> &state,
      const Input<LocalScalar> &input) const {
    PositionState<LocalScalar> input_state = PositionState<LocalScalar>::Zero();
    input_state.template topRows<kNumVelocityStates>() = state;
    return Dynamics(input_state, input).template topRows<kNumVelocityStates>();
  }

  std::pair<PositionStateSquare<>, PositionBMatrix<>> LinearizedDynamics(
      const PositionState<> &state, const Input<> &input) const {
    DynamicsFunctor functor(*this);
    Eigen::Matrix<Scalar, kNumPositionStates + kNumInputs, 1> parameters;
    parameters.template topRows<kNumPositionStates>() = state;
    parameters.template bottomRows<kNumInputs>() = input;
    const Eigen::Matrix<Scalar, kNumPositionStates,
                        kNumPositionStates + kNumInputs>
        jacobian =
            AutoDiffJacobian<Scalar, DynamicsFunctor,
                             kNumPositionStates + kNumInputs,
                             kNumPositionStates>::Jacobian(functor, parameters);
    return {
        jacobian.template block<kNumPositionStates, kNumPositionStates>(0, 0),
        jacobian.template block<kNumPositionStates, kNumInputs>(
            0, kNumPositionStates)};
  }

  std::pair<VelocityStateSquare<>, VelocityBMatrix<>>
  LinearizedVelocityDynamics(const VelocityState<> &state,
                             const Input<> &input) const {
    PositionState<> position_state = PositionState<>::Zero();
    position_state.template topRows<kNumVelocityStates>() = state;
    auto position_dynamics = LinearizedDynamics(position_state, input);
    return {
        position_dynamics.first
            .template block<kNumVelocityStates, kNumVelocityStates>(0, 0),
        position_dynamics.second.template block<kNumVelocityStates, kNumInputs>(
            0, 0)};
  }

 private:
  // Wrapper to provide an operator() for the dynamisc class that allows it to
  // be used by the auto-differentiation code.
  class DynamicsFunctor {
   public:
    DynamicsFunctor(const SimplifiedDynamics &dynamics) : dynamics_(dynamics) {}

    template <typename LocalScalar>
    Eigen::Matrix<LocalScalar, kNumPositionStates, 1> operator()(
        const Eigen::Map<const Eigen::Matrix<
            LocalScalar, kNumPositionStates + kNumInputs, 1>>
            input) const {
      return dynamics_.Dynamics(
          PositionState<LocalScalar>(
              input.template topRows<kNumPositionStates>()),
          Input<LocalScalar>(input.template bottomRows<kNumInputs>()));
    }

   private:
    const SimplifiedDynamics &dynamics_;
  };

  // Represents the dynamics of an individual module.
  class ModuleDynamics {
   public:
    ModuleDynamics(const Parameters &robot_params, const size_t module_index)
        : robot_params_(robot_params), module_index_(module_index) {
      CHECK_LT(module_index_, robot_params_.modules.size());
    }

    // This returns the portions of the derivative of state that are due to the
    // individual module. The result from this function should be able to be
    // naively summed with the dynamics for each other module plus some global
    // dynamics (which take care of that e.g. xdot = vx) and give you the
    // overall dynamics of the system.
    template <typename LocalScalar>
    PositionState<LocalScalar> PartialDynamics(
        const PositionState<LocalScalar> &state,
        const Input<LocalScalar> &input) const {
      PositionState<LocalScalar> Xdot = PositionState<LocalScalar>::Zero();

      Xdot(ThetasIdx()) = state(OmegasIdx());

      // Steering dynamics for an individual module assume ~zero friction,
      // and thus ~the only inertia is from the motor rotor itself.
      // torque_motor = stall_torque / stall_current * current
      // accel_motor = torque_motor / (motor_inertia + extra_steer_inertia *
      //                               steer_ratio)
      // accel_steer = accel_motor * steer_ratio
      const Motor &steer_motor = module_params().steer_motor;
      const LocalScalar steer_motor_accel =
          input(IsIdx()) *
          static_cast<Scalar>(module_params().steer_ratio *
                              steer_motor.stall_torque /
                              (steer_motor.stall_current *
                               (steer_motor.motor_inertia +
                                module_params().steer_ratio *
                                    module_params().extra_steer_inertia)));

      // For the impacts of the modules on the overall robot
      // dynamics (X, Y, and theta acceleration), we calculate the forces
      // generated by the module and then apply them. These forces come from
      // two effects in this model:
      // 1. Slip angle of the module (dependent on the current robot velocity &
      //    module steer angle).
      // 2. Drive torque from the module (dependent on the current drive
      //    current and module steer angle).
      // We assume no torque is generated from e.g. the wheel resisting the
      // steering motion.
      //
      // clang-format off
       //
       // For slip angle we have:
       // wheel_velocity = R(-theta - theta_steer) * (
       //    robot_vel + omega.cross(R(theta) * module_position))
       // slip_angle = -atan2(wheel_velocity)
       // slip_force = slip_angle_coefficient * slip_angle
       // slip_force_direction = theta + theta_steer + pi / 2
       // force_x = slip_force * cos(slip_force_direction)
       // force_y = slip_force * sin(slip_force_direction)
       // accel_* = force_* / mass
       // # And now calculate torque from slip angle.
       // torque_vec = module_position.cross([slip_force * cos(theta_steer + pi / 2),
       //                                     slip_force * sin(theta_steer + pi / 2),
       //                                     0.0])
       // torque_vec = module_position.cross([slip_force * -sin(theta_steer),
       //                                     slip_force * cos(theta_steer),
       //                                     0.0])
       // robot_torque = torque_vec.z()
       //
       // For drive torque we have:
       // drive_force = (drive_current * stall_torque / stall_current) / drive_ratio
       // drive_force_direction = theta + theta_steer
       // force_x = drive_force * cos(drive_force_direction)
       // force_y = drive_force * sin(drive_force_direction)
       // torque_vec = drive_force * module_position.cross([cos(theta_steer),
       //                                                   sin(theta_steer),
       //                                                   0.0])
       // torque = torque_vec.z()
       //
      // clang-format on

      const Eigen::Matrix<Scalar, 3, 1> module_position{
          {module_params().position.x()},
          {module_params().position.y()},
          {0.0}};
      const LocalScalar theta = state(kTheta);
      const LocalScalar theta_steer = state(ThetasIdx());
      const Eigen::Matrix<LocalScalar, 3, 1> wheel_velocity_in_global_frame =
          Eigen::Matrix<LocalScalar, 3, 1>(state(kVx), state(kVy),
                                           static_cast<LocalScalar>(0.0)) +
          (Eigen::Matrix<LocalScalar, 3, 1>(static_cast<LocalScalar>(0.0),
                                            static_cast<LocalScalar>(0.0),
                                            state(kOmega))
               .cross(Eigen::AngleAxis<LocalScalar>(
                          theta, Eigen::Matrix<LocalScalar, 3, 1>::UnitZ()) *
                      module_position));
      const Eigen::Matrix<LocalScalar, 3, 1> wheel_velocity_in_wheel_frame =
          Eigen::AngleAxis<LocalScalar>(
              -theta - theta_steer, Eigen::Matrix<LocalScalar, 3, 1>::UnitZ()) *
          wheel_velocity_in_global_frame;
      // The complicated dynamics use some obnoxious-looking functions to
      // try to approximate how the slip angle behaves a low speeds to better
      // condition the dynamics. Because I couldn't be bothered to copy those
      // dynamics, instead just bias the slip angle to zero at low speeds.
      const LocalScalar wheel_speed = wheel_velocity_in_wheel_frame.norm();
      const Scalar start_speed = 0.1;
      const LocalScalar heading_truth_proportion = -expm1(
          /*arbitrary large number=*/static_cast<Scalar>(-100.0) *
          (wheel_speed - start_speed));
      const LocalScalar wheel_heading =
          (wheel_speed < start_speed)
              ? static_cast<LocalScalar>(0.0)
              : heading_truth_proportion *
                    atan2(wheel_velocity_in_wheel_frame.y(),
                          wheel_velocity_in_wheel_frame.x());

      // We wrap slip_angle with a sin() not because there is actually a sin()
      // in the real math but rather because we need to smoothly and correctly
      // handle slip angles between pi / 2 and 3 * pi / 2.
      const LocalScalar slip_angle = sin(-wheel_heading);
      const LocalScalar slip_force =
          module_params().slip_angle_coefficient * slip_angle;
      const LocalScalar slip_force_direction =
          theta + theta_steer + static_cast<Scalar>(M_PI_2);
      const Eigen::Matrix<LocalScalar, 3, 1> slip_force_vec =
          slip_force * UnitYawVector<LocalScalar>(slip_force_direction);
      const LocalScalar slip_torque =
          module_position
              .cross(slip_force *
                     UnitYawVector<LocalScalar>(theta_steer +
                                                static_cast<Scalar>(M_PI_2)))
              .z();

      // drive torque calculations
      const Motor &drive_motor = module_params().drive_motor;
      const LocalScalar drive_force =
          input(IdIdx()) * static_cast<Scalar>(drive_motor.stall_torque /
                                               drive_motor.stall_current /
                                               module_params().drive_ratio);
      const Eigen::Matrix<LocalScalar, 3, 1> drive_force_vec =
          drive_force * UnitYawVector<LocalScalar>(theta + theta_steer);
      const LocalScalar drive_torque =
          drive_force *
          module_position.cross(UnitYawVector<LocalScalar>(theta_steer)).z();
      // We add in an aligning force on the wheels primarily to help provide a
      // bit of impetus to the controllers/solvers to discourage aggressive
      // slip angles. If we do not include this, then the dynamics make it look
      // like there are no losses to using extremely aggressive slip angles.
      const LocalScalar wheel_alignment_accel =
          -module_params().slip_angle_alignment_coefficient * slip_angle;

      Xdot(OmegasIdx()) = steer_motor_accel + wheel_alignment_accel;
      // Sum up all the forces.
      Xdot(kVx) =
          (slip_force_vec.x() + drive_force_vec.x()) / robot_params_.mass;
      Xdot(kVy) =
          (slip_force_vec.y() + drive_force_vec.y()) / robot_params_.mass;
      Xdot(kOmega) =
          (slip_torque + drive_torque) / robot_params_.moment_of_inertia;

      return Xdot;
    }

   private:
    template <typename LocalScalar>
    Eigen::Matrix<LocalScalar, 3, 1> UnitYawVector(LocalScalar yaw) const {
      return Eigen::Matrix<LocalScalar, 3, 1>{
          {static_cast<LocalScalar>(cos(yaw))},
          {static_cast<LocalScalar>(sin(yaw))},
          {static_cast<LocalScalar>(0.0)}};
    }
    size_t ThetasIdx() const { return kThetas0 + 2 * module_index_; }
    size_t OmegasIdx() const { return kOmegas0 + 2 * module_index_; }
    size_t IsIdx() const { return Inputs::kIs0 + 2 * module_index_; }
    size_t IdIdx() const { return Inputs::kId0 + 2 * module_index_; }

    const ModuleParams &module_params() const {
      return robot_params_.modules[module_index_];
    }

    const Parameters robot_params_;

    const size_t module_index_;
  };

  Parameters params_;
  std::vector<ModuleDynamics> module_dynamics_;
};

}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_SIMPLIFIED_DYNAMICS_H_
