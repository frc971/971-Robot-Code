#ifndef frc971_CONTROL_LOOPS_SWERVE_LINEARIZED_CONTROLLER_H_
#define frc971_CONTROL_LOOPS_SWERVE_LINEARIZED_CONTROLLER_H_
#include <memory>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include <Eigen/Dense>

#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/dlqr.h"
#include "frc971/control_loops/jacobian.h"
#include "frc971/control_loops/swerve/dynamics.h"
#include "frc971/control_loops/swerve/linearization_utils.h"

ABSL_DECLARE_FLAG(bool, use_slicot);
ABSL_DECLARE_FLAG(int, dare_iterations);

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
  typedef Eigen::Matrix<Scalar, kNumInputs, NStates> GainMatrix;
  typedef DynamicsInterface<Scalar, NStates, kNumInputs> Dynamics;
  typedef Dynamics::LinearDynamics LinearDynamics;

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

  // Debug information for a given cycle of the controller.
  struct ControllerDebug {
    // Feedforward input which we provided.
    Input U_ff;
    // Calculated feedback input to provide.
    Input U_feedback;
    Eigen::Matrix<Scalar, kNumInputs, NStates> feedback_contributions;
    int sb02od_exit_code;
  };

  struct ControllerResult {
    // Control input to provide to the robot.
    Input U;
    ControllerDebug debug;
  };

  LinearizedController(Parameters params) : params_(std::move(params)) {}

  const LinearDynamics LinearizeDynamics(const State &X, const Input &U) const {
    return params_.dynamics->LinearizeDynamics(X, U);
  }

  // Runs the controller for a given iteration, relinearizing the dynamics about
  // the provided current state X, attempting to control the robot to the
  // desired goal state.
  // The U_ff input will be added into the returned control input.
  ControllerResult RunController(const State &X, const State &goal,
                                 Input U_ff) {
    auto start_time = aos::monotonic_clock::now();
    const LinearDynamics continuous_dynamics = LinearizeDynamics(X, U_ff);
    auto linearization_time = aos::monotonic_clock::now();
    LinearDynamics discrete_dynamics;
    frc971::controls::C2D(continuous_dynamics.A, continuous_dynamics.B,
                          params_.dt, &discrete_dynamics.A,
                          &discrete_dynamics.B);
    auto c2d_time = aos::monotonic_clock::now();
    VLOG(2) << "Controllability of dynamics (ideally should be " << NStates
            << "): "
            << frc971::controls::Controllability(discrete_dynamics.A,
                                                 discrete_dynamics.B);
    int sb02od_exit_code = -1;
    GainMatrix K = SolveDare(
        discrete_dynamics.A, discrete_dynamics.B, params_.Q, params_.R,
        absl::GetFlag(FLAGS_use_slicot) ? DareSolver::Slicot
                                        : DareSolver::IterativeApproximation,
        &sb02od_exit_code);
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
                      .feedback_contributions = feedback_contributions,
                      .sb02od_exit_code = sb02od_exit_code}};
  }

  // Specifies what version of the DARE solver to use when attempting to solve
  // for the LQR gains. The SLICOT solver finds an actual solution the the DARE,
  // but takes longer and can be less stable, while the iterative method does a
  // more alzy approximation which effectively performs a finite-horizon LQR.
  enum class DareSolver { Slicot, IterativeApproximation };

  GainMatrix SolveDare(const StateSquare &A, const BMatrix &B,
                       const StateSquare &Q, const InputSquare &R,
                       DareSolver solver, int *sb02od_exit_code) {
    if (solver == DareSolver::Slicot) {
      Eigen::Matrix<double, kNumInputs, NStates> K;
      // TODO(james): Swap this to a cheaper DARE solver; we should probably
      // just do something like we do in Trajectory::CalculatePathGains for the
      // tank spline controller where we approximate the infinite-horizon DARE
      // solution by doing a finite-horizon LQR.
      *sb02od_exit_code = (frc971::controls::dlqr<NStates, kNumInputs>(
          A.template cast<double>(), B.template cast<double>(),
          Q.template cast<double>(), R.template cast<double>(), &K, nullptr));
      if (*sb02od_exit_code == 0) {
        last_K_ = K.template cast<Scalar>();
      }
      return last_K_;
    } else {
      // This mode effectively approximates the solution to the DARE as a
      // finite-horizon LQR
      // (https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Finite-horizon,_discrete-time).
      // For sufficiently large iteration counts this should be correct,
      // although it will be much less efficient than a proper solver.
      StateSquare P = Q;
      const int kDareIters = absl::GetFlag(FLAGS_dare_iterations);
      BMatrix APB;
      InputSquare RBPBinv;
      for (int ii = 0; ii < kDareIters; ++ii) {
        const StateSquare AP = A.transpose() * P;
        CHECK(AP.allFinite());
        APB = AP * B;
        CHECK(APB.allFinite());
        RBPBinv = (R + B.transpose() * P * B).inverse();
        P = AP * A - APB * RBPBinv * APB.transpose() + Q;
      }
      CHECK(P.allFinite());
      CHECK_LT(P.norm(), 1e30) << "LQR calculations became unstable.";
      return (R + B.transpose() * P * B).inverse() *
             (A.transpose() * P * B).transpose();
    }
  }

 private:
  const Parameters params_;
  GainMatrix last_K_ = GainMatrix::Zero();
};

}  // namespace frc971::control_loops::swerve
#endif  // frc971_CONTROL_LOOPS_SWERVE_LINEARIZED_CONTROLLER_H_
