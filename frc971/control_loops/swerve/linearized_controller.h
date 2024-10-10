#include <memory>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include <Eigen/Dense>

#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/dlqr.h"
#include "frc971/control_loops/jacobian.h"
#include "frc971/control_loops/swerve/dynamics.h"
#include "frc971/control_loops/swerve/linearization_utils.h"

namespace frc971::control_loops::swerve {

// Provides a simple LQR controller that takes a non-linear system, linearizes
// the dynamics at each timepoint, recalculates the LQR gains for those
// dynamics, and calculates the relevant feedback inputs to provide.
template <int NStates, typename Scalar = double>
class LinearizedController {
 public:
  typedef Eigen::Matrix<Scalar, NStates, 1> State;
  typedef Eigen::Matrix<Scalar, NStates, NStates> StateSquare;
  typedef Eigen::Matrix<Scalar, kNumInputs, 1> Input;
  typedef Eigen::Matrix<Scalar, kNumInputs, kNumInputs> InputSquare;
  typedef Eigen::Matrix<Scalar, NStates, kNumInputs> BMatrix;
  typedef DynamicsInterface<State, Input> Dynamics;

  struct Parameters {
    // State cost matrix.
    StateSquare Q;
    // Input cost matrix.
    InputSquare R;
    // period at which the controller is called.
    std::chrono::nanoseconds dt;
    // The dynamics to use.
    // TODO(james): I wrote this before creating the auto-differentiation
    // functions; we should swap to the auto-differentiation, since the
    // numerical linearization is one of the bigger timesinks in this controller
    // right now.
    std::unique_ptr<Dynamics> dynamics;
  };

  // Represents the linearized dynamics of the system.
  struct LinearDynamics {
    StateSquare A;
    BMatrix B;
  };

  // Debug information for a given cycle of the controller.
  struct ControllerDebug {
    // Feedforward input which we provided.
    Input U_ff;
    // Calculated feedback input to provide.
    Input U_feedback;
    Eigen::Matrix<Scalar, kNumInputs, NStates> feedback_contributions;
  };

  struct ControllerResult {
    // Control input to provide to the robot.
    Input U;
    ControllerDebug debug;
  };

  LinearizedController(Parameters params) : params_(std::move(params)) {}

  // Runs the controller for a given iteration, relinearizing the dynamics about
  // the provided current state X, attempting to control the robot to the
  // desired goal state.
  // The U_ff input will be added into the returned control input.
  ControllerResult RunController(const State &X, const State &goal,
                                 Input U_ff) {
    auto start_time = aos::monotonic_clock::now();
    // TODO(james): Swap this to the auto-diff methods; this is currently about
    // a third of the total time spent in this method when run on the roborio.
    const struct LinearDynamics continuous_dynamics =
        LinearizeDynamics(X, U_ff);
    auto linearization_time = aos::monotonic_clock::now();
    struct LinearDynamics discrete_dynamics;
    frc971::controls::C2D(continuous_dynamics.A, continuous_dynamics.B,
                          params_.dt, &discrete_dynamics.A,
                          &discrete_dynamics.B);
    auto c2d_time = aos::monotonic_clock::now();
    VLOG(2) << "Controllability of dynamics (ideally should be " << NStates
            << "): "
            << frc971::controls::Controllability(discrete_dynamics.A,
                                                 discrete_dynamics.B);
    Eigen::Matrix<Scalar, kNumInputs, NStates> K;
    Eigen::Matrix<Scalar, NStates, NStates> S;
    // TODO(james): Swap this to a cheaper DARE solver; we should probably just
    // do something like we do in Trajectory::CalculatePathGains for the tank
    // spline controller where we approximate the infinite-horizon DARE solution
    // by doing a finite-horizon LQR.
    // Currently the dlqr call represents ~60% of the time spent in the
    // RunController() method.
    frc971::controls::dlqr(discrete_dynamics.A, discrete_dynamics.B, params_.Q,
                           params_.R, &K, &S);
    auto dlqr_time = aos::monotonic_clock::now();
    const Input U_feedback = K * (goal - X);
    const Input U = U_ff + U_feedback;
    Eigen::Matrix<Scalar, kNumInputs, NStates> feedback_contributions;
    for (int state_idx = 0; state_idx < NStates; ++state_idx) {
      feedback_contributions.col(state_idx) =
          K.col(state_idx) * (goal - X)(state_idx);
    }
    VLOG(2) << "linearization time "
            << aos::time::DurationInSeconds(linearization_time - start_time)
            << " c2d time "
            << aos::time::DurationInSeconds(c2d_time - linearization_time)
            << " dlqr time "
            << aos::time::DurationInSeconds(dlqr_time - c2d_time);
    return {.U = U,
            .debug = {.U_ff = U_ff,
                      .U_feedback = U_feedback,
                      .feedback_contributions = feedback_contributions}};
  }

  LinearDynamics LinearizeDynamics(const State &X, const Input &U) {
    return {.A = NumericalJacobianX(*params_.dynamics, X, U),
            .B = NumericalJacobianU(*params_.dynamics, X, U)};
  }

 private:
  const Parameters params_;
};

}  // namespace frc971::control_loops::swerve
