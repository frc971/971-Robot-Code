#ifndef FRC971_CONTROL_LOOPS_SWERVE_AUTO_DIFF_JACOBIAN_H_
#define FRC971_CONTROL_LOOPS_SWERVE_AUTO_DIFF_JACOBIAN_H_
#include "include/ceres/tiny_solver.h"
#include "include/ceres/tiny_solver_autodiff_function.h"

namespace frc971::control_loops::swerve {
// Class to conveniently scope a function that makes use of Ceres'
// autodifferentiation methods to calculate the jacobian of the provided method.
// Template parameters:
// Scalar: scalar type to use (typically double or float; this is used for
//    allowing you to control what precision you use; you cannot use this
//    with ceres Jets because this has to use Jets internally).
// Function: The type of the function itself. A Function f must be callable as
//    Eigen::Matrix<Scalar, kNumOutputs, 1> = f(Eigen::Matrix<Scalar,
//    kNumInputs, 1>{});
template <typename Scalar, typename Function, size_t kNumInputs,
          size_t kNumOutputs>
class AutoDiffJacobian {
 public:
  // Calculates the jacobian of the provided method, function, at the provided
  // input X.
  static Eigen::Matrix<Scalar, kNumOutputs, kNumInputs> Jacobian(
      const Function &function, const Eigen::Matrix<Scalar, kNumInputs, 1> &X) {
    AutoDiffCeresFunctor ceres_functor(function);
    TinySolverFunctor tiny_solver(ceres_functor);
    // residual is unused, it's just a place to store the evaluated function at
    // the current state/input.
    Eigen::Matrix<Scalar, kNumOutputs, 1> residual;
    Eigen::Matrix<Scalar, kNumOutputs, kNumInputs> jacobian;
    tiny_solver(X.data(), residual.data(), jacobian.data());
    return jacobian;
  }

 private:
  // Borrow the TinySolver's auto-differentiation execution for use here to
  // calculate the linearized dynamics. We aren't actually doing any solving,
  // just letting it do the jacobian calculation for us.
  // As such, construct a "residual" function whose residuals are just the
  // derivative of the state and whose parameters are the stacked state + input.
  class AutoDiffCeresFunctor {
   public:
    AutoDiffCeresFunctor(const Function &function) : function_(function) {}
    template <typename ScalarT>
    bool operator()(const ScalarT *const parameters,
                    ScalarT *const residuals) const {
      const Eigen::Map<const Eigen::Matrix<ScalarT, kNumInputs, 1>>
          eigen_parameters(parameters);
      Eigen::Map<Eigen::Matrix<ScalarT, kNumOutputs, 1>> eigen_residuals(
          residuals);
      eigen_residuals = function_(eigen_parameters);
      return true;
    }

   private:
    const Function &function_;
  };
  typedef ceres::TinySolverAutoDiffFunction<AutoDiffCeresFunctor, kNumOutputs,
                                            kNumInputs, Scalar>
      TinySolverFunctor;
};
}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_AUTO_DIFF_JACOBIAN_H_
