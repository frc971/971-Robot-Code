#ifndef FRC971_CONTROL_LOOPS_SWERVE_LINEAR_VELOCITY_CONTROLLER_H_
#define FRC971_CONTROL_LOOPS_SWERVE_LINEAR_VELOCITY_CONTROLLER_H_
#include "frc971/control_loops/swerve/inverse_kinematics.h"
#include "frc971/control_loops/swerve/linearized_controller.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"

namespace frc971::control_loops::swerve {

// This implements a basic linear controller to control the velocity of a swerve
// drivebase.
// The states of this controller are just the velocity states of the
// SimplifiedDynamics class, which does not have any sort of integral terms or
// the such, so unmodeled frictions will result in this controller not causing
// the drivebase to actually get up to speed.
class LinearVelocityController {
 public:
  using Scalar = float;
  using Dynamics = SimplifiedDynamics<Scalar>;
  static constexpr size_t kNumStates = Dynamics::kNumVelocityStates;
  using Controller = LinearizedController<kNumStates, Scalar>;
  using States = Dynamics::States;
  using Inputs = Dynamics::Inputs;
  using State = Controller::State;
  using Input = Controller::Input;
  using StateSquare = Controller::StateSquare;
  using InputSquare = Controller::InputSquare;
  using BMatrix = Controller::BMatrix;
  using Parameters = Controller::Parameters;
  using DynamicsParameters = Dynamics::Parameters;

  using VirtualDynamics = Dynamics::VirtualVelocityDynamics;

  // TODO(james): STop hard-coding dt.
  static constexpr std::chrono::milliseconds kDt{10};

  // Represents the goal states that we will typically control to.
  struct Goal {
    Scalar vx;
    Scalar vy;
    Scalar omega;
  };

  // Set of weights to fill out the LQR cost matrices with.
  struct ControllerWeights {
    // Costs for the steer module angles and steer rates.
    double thetas_q;
    double omegas_q;
    // Cost for omega_d(constant)
    double omegad_q = 0.0;
    // Factor to cost slip(added to the omegad row)
    double k_slip = 0.0;
    // Cost for the robot vx/vy.
    double vel_q;
    // Cost for the robot heading goal.
    double theta_q;
    // Cost for the robot yaw rate goal.
    double omega_q;
    // Costs for the currents on the steer and drive motors.
    double steer_current_r;
    double drive_current_r;
  };

  // Debug information associated with a given controller iteration; most of
  // this ends up coming from the LinearizedController class.
  struct ControllerDebug {
    Input U_ff;
    Input U_feedback;
    Eigen::Matrix<Scalar, kNumInputs, kNumStates> feedback_contributions;
    State goal;
    int sb02od_exit_code;
  };

  struct ControllerResult {
    Input U;
    ControllerDebug debug;
  };

  static Parameters MakeParameters(
      const ControllerWeights weights,
      const DynamicsParameters &params = MakeDynamicsParameters());
  static DynamicsParameters MakeDynamicsParameters();

  LinearVelocityController(
      Parameters params,
      const DynamicsParameters &dynamics_params = MakeDynamicsParameters());

  // Runs the inverse kinematics to generate a goal state and then calls
  // RunRawController().
  ControllerResult RunController(const State &X, const Goal &goal);
  // Runs the controller for the current state and goal, adding in the provided
  // feedforwards to pass through to the output.
  ControllerResult RunRawController(const State &X, const State &goal,
                                    const Input &U_ff);

  // Generates a full State vector with just the values corresonding with the
  // states in goal set (all other entries will be set to zero). This is
  // suitable for passing into inverse kinematics/dynamics calculators.
  static State MakeGoal(const Goal &goal);

 private:
  InverseKinematics<Scalar> inverse_kinematics_;
  Controller controller_;
};

}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_LINEAR_VELOCITY_CONTROLLER_H_
