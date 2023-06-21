#ifndef FRC971_SOLVERS_SPARSE_CONVEX_H_
#define FRC971_SOLVERS_SPARSE_CONVEX_H_

#include <sys/types.h>
#include <unistd.h>

#include <Eigen/Sparse>
#include <iomanip>

#include "glog/logging.h"

namespace frc971 {
namespace solvers {

// TODO(austin): Steal JET from Ceres to generate the derivatives easily and
// quickly?
//
// States is the number of inputs to the optimization problem.
// M is the number of inequality constraints.
// N is the number of equality constraints.
class SparseConvexProblem {
 public:
  size_t states() const { return states_; }
  size_t inequality_constraints() const { return inequality_constraints_; }
  size_t equality_constraints() const { return equality_constraints_; }

  // Returns the function to minimize and it's derivatives.
  virtual double f0(Eigen::Ref<const Eigen::VectorXd> X) const = 0;
  // TODO(austin): Should the jacobian be sparse?
  virtual Eigen::SparseMatrix<double> df0(
      Eigen::Ref<const Eigen::VectorXd> X) const = 0;
  virtual Eigen::SparseMatrix<double> ddf0(
      Eigen::Ref<const Eigen::VectorXd> X) const = 0;

  // Returns the constraints f(X) < 0, and their derivative.
  virtual Eigen::VectorXd f(Eigen::Ref<const Eigen::VectorXd> X) const = 0;
  virtual Eigen::SparseMatrix<double> df(
      Eigen::Ref<const Eigen::VectorXd> X) const = 0;

  // Returns the equality constraints of the form A x = b
  virtual Eigen::SparseMatrix<double> A() const = 0;
  virtual Eigen::VectorXd b() const = 0;

 protected:
  SparseConvexProblem(size_t states, size_t inequality_constraints,
                      size_t equality_constraints)
      : states_(states),
        inequality_constraints_(inequality_constraints),
        equality_constraints_(equality_constraints) {}

 private:
  size_t states_;
  size_t inequality_constraints_;
  size_t equality_constraints_;
};

// Implements a Primal-Dual Interior point method convex solver.
// See 11.7 of https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf
//
// States is the number of inputs to the optimization problem.
// M is the number of inequality constraints.
// N is the number of equality constraints.
class SparseSolver {
 public:
  // Ratio to require the cost to decrease when line searching.
  static constexpr double kAlpha = 0.05;
  // Line search step parameter.
  static constexpr double kBeta = 0.5;
  static constexpr double kMu = 2.0;
  // Terminal condition for the primal problem (equality constraints) and dual
  // (gradient + inequality constraints).
  static constexpr double kEpsilonF = 1e-6;
  // Terminal condition for nu, the surrogate duality gap.
  static constexpr double kEpsilon = 1e-6;

  // Solves the problem given a feasible initial solution.
  Eigen::VectorXd Solve(const SparseConvexProblem &problem,
                        Eigen::Ref<const Eigen::VectorXd> X_initial);

 private:
  // Class to hold all the derivataves and function evaluations.
  struct Derivatives {
    size_t states() const { return hessian.rows(); }
    size_t inequality_constraints() const { return f.rows(); }
    size_t equality_constraints() const { return Axmb.rows(); }

    Eigen::SparseMatrix<double> gradient;
    Eigen::SparseMatrix<double> hessian;

    // Inequality function f
    Eigen::VectorXd f;
    // df
    Eigen::SparseMatrix<double> df;

    // ddf is assumed to be 0 because for the linear constraint distance
    // function we are using, it is actually 0, and by assuming it is zero
    // rather than passing it through as 0 to the solver, we can save enough CPU
    // to make it worth it.

    // A
    Eigen::SparseMatrix<double> A;
    // Ax - b
    Eigen::VectorXd Axmb;
  };

  // Computes all the values for the given problem at the given state.
  Derivatives ComputeDerivative(
      const SparseConvexProblem &problem,
      const Eigen::Ref<const Eigen::VectorXd> y);

  // Computes Rt at the given state and with the given t_inverse.  See 11.53 of
  // cvxbook.pdf.
  Eigen::VectorXd Rt(
      const Derivatives &derivatives,
      Eigen::VectorXd y, double t_inverse);

  // Prints out all the derivatives with VLOG at the provided verbosity.
  void PrintDerivatives(
      const Derivatives &derivatives,
      const Eigen::Ref<const Eigen::VectorXd> y,
      std::string_view prefix, int verbosity);
};

}  // namespace solvers
}  // namespace frc971

#endif  // FRC971_SOLVERS_SPARSE_CONVEX_H_
