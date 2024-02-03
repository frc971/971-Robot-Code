#include "frc971/control_loops/catapult/mpc_problem_generator.h"
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

namespace frc971 {
namespace control_loops {
namespace catapult {

class CatapultController {
 public:
  CatapultController(StateFeedbackPlant<2, 1, 1> plant, size_t horizon);

  // Starts back over at the first controller.
  void Reset();

  // Updates our current and final states for the current controller.
  void SetState(Eigen::Matrix<double, 2, 1> X_initial,
                Eigen::Matrix<double, 2, 1> X_final);

  // Solves!  Returns true if the solution converged and osqp was happy.
  bool Solve();

  // Returns the time in seconds it last took Solve to run.
  double solve_time() const { return solve_time_; }

  // Returns the horizon of the current controller.
  size_t current_horizon() const {
    if (current_controller_ >= problems_.size()) {
      return 0u;
    } else {
      return problems_[current_controller_]->horizon();
    }
  }

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

}  // namespace catapult
}  // namespace control_loops
}  // namespace frc971