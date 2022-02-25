#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_

#include "Eigen/Dense"
#include "frc971/control_loops/state_feedback_loop.h"
#include "glog/logging.h"
#include "osqp++.h"
#include "y2022/constants.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_position_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace catapult {

// MPC problem for a specified horizon.  This contains all the state for the
// solver, setters to modify the current and target state, and a way to fetch
// the solution.
class MPCProblem {
 public:
  MPCProblem(size_t horizon,
             Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P,
             Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q,
             Eigen::Matrix<double, 2, 2> Af,
             Eigen::Matrix<double, Eigen::Dynamic, 2> final_q);

  MPCProblem(MPCProblem const &) = delete;
  void operator=(MPCProblem const &x) = delete;

  // Sets the current and final state.  This keeps the problem in tact and
  // doesn't recreate it, so it will be fast.
  void SetState(Eigen::Matrix<double, 2, 1> X_initial,
                Eigen::Matrix<double, 2, 1> X_final);

  // Solves our problem.
  bool Solve();

  double solve_time() const { return solve_time_; }

  // Returns the solution that the solver found when Solve was last called.
  double U(size_t i) const { return solver_.primal_solution()(i); }

  // Returns the number of U's to be solved.
  size_t horizon() const { return horizon_; }

  // Warm starts the optimizer with the provided solution to make it solve
  // faster.
  void WarmStart(const MPCProblem &p);

 private:
  // The number of u's to solve for.
  const size_t horizon_;

  // The problem statement variables needed by SetState to update q.
  const Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q_;
  const Eigen::Matrix<double, 2, 2> Af_;
  const Eigen::Matrix<double, Eigen::Dynamic, 2> final_q_;

  Eigen::Matrix<double, 2, 1> X_initial_;
  Eigen::Matrix<double, 2, 1> X_final_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> objective_vector_;

  // Solver state.
  osqp::OsqpInstance instance_;
  osqp::OsqpSolver solver_;
  osqp::OsqpSettings settings_;

  double solve_time_ = 0;
};

// Decently efficient problem generator for multiple horizons given a max
// horizon to solve for.
//
// The math is documented in mpc.tex
class CatapultProblemGenerator {
 public:
  // Builds a problem generator for the specified max horizon and caches a lot
  // of the state.
  CatapultProblemGenerator(size_t horizon);

  // Returns the maximum horizon.
  size_t horizon() const { return horizon_; }

  // Makes a problem for the specificed horizon.
  std::unique_ptr<MPCProblem> MakeProblem(size_t horizon);

  // Returns the P and Q matrices for the problem statement.
  //   cost = 0.5 X.T P X + q.T X
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P(size_t horizon);
  const Eigen::Matrix<double, Eigen::Dynamic, 1> q(
      size_t horizon, Eigen::Matrix<double, 2, 1> X_initial,
      Eigen::Matrix<double, 2, 1> X_final);

 private:
  const Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q(size_t horizon);

  const Eigen::Matrix<double, 2, 2> Af(size_t horizon);
  const Eigen::Matrix<double, 2, Eigen::Dynamic> Bf(size_t horizon);
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Pi(
      size_t horizon);

  // These functions are used in the constructor to build up the matrices below.
  Eigen::Matrix<double, Eigen::Dynamic, 2> MakeAs();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeBs();
  Eigen::Matrix<double, Eigen::Dynamic, 1> Makem();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeM();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeW();
  Eigen::Matrix<double, Eigen::Dynamic, 1> Makew();
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> MakePi();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeP();

  const StateFeedbackPlant<2, 1, 1> plant_;
  const size_t horizon_;

  const Eigen::DiagonalMatrix<double, 2> Q_final_;

  const Eigen::Matrix<double, Eigen::Dynamic, 2> As_;
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Bs_;
  const Eigen::Matrix<double, Eigen::Dynamic, 1> m_;
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M_;

  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_;
  const Eigen::Matrix<double, Eigen::Dynamic, 1> w_;
  const Eigen::DiagonalMatrix<double, Eigen::Dynamic> Pi_;

  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> WM_;
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Wmpw_;
};

// A class to hold all the state needed to manage the catapult MPC solvers for
// repeated shots.
//
// The solver may take a couple of cycles to get everything converged and ready.
// The flow is as follows:
//  1) Reset() the state for the new problem.
//  2) Update to the current state with SetState()
//  3) Call Solve().  This will return true if it is ready to be executed, false
//     if it needs more iterations to fully converge.
//  4) Next() returns the current optimal control output and advances the
//     pointers to the next problem.
//  5) Go back to 2 for the next cycle.
class CatapultController {
 public:
  CatapultController(size_t horizon);

  // Starts back over at the first controller.
  void Reset();

  // Updates our current and final states for the current controller.
  void SetState(Eigen::Matrix<double, 2, 1> X_initial,
                Eigen::Matrix<double, 2, 1> X_final);

  // Solves!  Returns true if the solution converged and osqp was happy.
  bool Solve();

  // Returns the time in seconds it last took Solve to run.
  double solve_time() const { return solve_time_; }

  // Returns the controller value if there is a controller to run, or nullopt if
  // we finished the last controller.  Advances the controller pointer to the
  // next controller and warms up the next controller.
  std::optional<double> Next();

  // Returns true if Next has been called and a controller has been used.  Reset
  // starts over.
  bool started() const { return current_controller_ != 0u; }

 private:
  CatapultProblemGenerator generator_;

  std::vector<std::unique_ptr<MPCProblem>> problems_;

  size_t current_controller_ = 0;
  double solve_time_ = 0.0;
};

// Class to handle transitioning between both the profiled subsystem and the MPC
// for shooting.
class Catapult {
 public:
  Catapult(const constants::Values &values)
      : catapult_(values.catapult.subsystem_params), catapult_mpc_(35) {}

  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

  // Resets all state for when WPILib restarts.
  void Reset() { catapult_.Reset(); }

  bool zeroed() const { return catapult_.zeroed(); }
  bool estopped() const { return catapult_.estopped(); }
  double solve_time() const { return catapult_mpc_.solve_time(); }

  bool mpc_active() const { return !use_profile_; }

  // Returns the number of shots taken.
  int shot_count() const { return shot_count_; }

  // Runs either the MPC or the profiled subsystem depending on if we are
  // shooting or not.  Returns the status.
  const flatbuffers::Offset<
      frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
  Iterate(const Goal *unsafe_goal, const Position *position,
          double *catapult_voltage, flatbuffers::FlatBufferBuilder *fbb);

 private:
  // TODO(austin): Prototype is just an encoder.  Catapult has both an encoder
  // and pot.  Switch back once we have a catapult.
  // PotAndAbsoluteEncoderSubsystem catapult_;
  PotAndAbsoluteEncoderSubsystem catapult_;

  catapult::CatapultController catapult_mpc_;

  enum CatapultState { PROFILE, FIRING, RESETTING };

  CatapultState catapult_state_ = CatapultState::PROFILE;

  bool last_firing_ = false;
  bool use_profile_ = true;

  int shot_count_ = 0;
};

}  // namespace catapult
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022

#endif  // Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_
