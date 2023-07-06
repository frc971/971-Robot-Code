#ifndef FRC971_SOLVERS_CONVEX_H_
#define FRC971_SOLVERS_CONVEX_H_

#include <sys/types.h>
#include <unistd.h>

#include <iomanip>

#include "absl/strings/str_join.h"
#include "glog/logging.h"
#include <Eigen/Dense>

namespace frc971 {
namespace solvers {

// TODO(austin): Steal JET from Ceres to generate the derivatives easily and
// quickly?
//
// States is the number of inputs to the optimization problem.
// M is the number of inequality constraints.
// N is the number of equality constraints.
template <size_t States, size_t M, size_t N>
class ConvexProblem {
 public:
  // Returns the function to minimize and it's derivatives.
  virtual double f0(
      Eigen::Ref<const Eigen::Matrix<double, States, 1>> X) const = 0;
  virtual Eigen::Matrix<double, States, 1> df0(
      Eigen::Ref<const Eigen::Matrix<double, States, 1>> X) const = 0;
  virtual Eigen::Matrix<double, States, States> ddf0(
      Eigen::Ref<const Eigen::Matrix<double, States, 1>> X) const = 0;

  // Returns the constraints f(X) < 0, and their derivative.
  virtual Eigen::Matrix<double, M, 1> f(
      Eigen::Ref<const Eigen::Matrix<double, States, 1>> X) const = 0;
  virtual Eigen::Matrix<double, M, States> df(
      Eigen::Ref<const Eigen::Matrix<double, States, 1>> X) const = 0;

  // Returns the equality constraints of the form A x = b
  virtual Eigen::Matrix<double, N, States> A() const = 0;
  virtual Eigen::Matrix<double, N, 1> b() const = 0;
};

// Implements a Primal-Dual Interior point method convex solver.
// See 11.7 of https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf
//
// States is the number of inputs to the optimization problem.
// M is the number of inequality constraints.
// N is the number of equality constraints.
template <size_t States, size_t M, size_t N>
class Solver {
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
  Eigen::Matrix<double, States, 1> Solve(
      const ConvexProblem<States, M, N> &problem,
      Eigen::Ref<const Eigen::Matrix<double, States, 1>> X_initial);

 private:
  // Class to hold all the derivataves and function evaluations.
  struct Derivatives {
    Eigen::Matrix<double, States, 1> gradient;
    Eigen::Matrix<double, States, States> hessian;

    // Inequality function f
    Eigen::Matrix<double, M, 1> f;
    // df
    Eigen::Matrix<double, M, States> df;

    // ddf is assumed to be 0 because for the linear constraint distance
    // function we are using, it is actually 0, and by assuming it is zero
    // rather than passing it through as 0 to the solver, we can save enough CPU
    // to make it worth it.

    // A
    Eigen::Matrix<double, N, States> A;
    // Ax - b
    Eigen::Matrix<double, N, 1> Axmb;
  };

  // Computes all the values for the given problem at the given state.
  Derivatives ComputeDerivative(
      const ConvexProblem<States, M, N> &problem,
      const Eigen::Ref<const Eigen::Matrix<double, States + M + N, 1>> y);

  // Computes Rt at the given state and with the given t_inverse.  See 11.53 of
  // cvxbook.pdf.
  Eigen::Matrix<double, States + M + N, 1> Rt(
      const Derivatives &derivatives,
      Eigen::Matrix<double, States + M + N, 1> y, double t_inverse);

  // Prints out all the derivatives with VLOG at the provided verbosity.
  void PrintDerivatives(
      const Derivatives &derivatives,
      const Eigen::Ref<const Eigen::Matrix<double, States + M + N, 1>> y,
      std::string_view prefix, int verbosity);
};

template <size_t States, size_t M, size_t N>
Eigen::Matrix<double, States + M + N, 1> Solver<States, M, N>::Rt(
    const Derivatives &derivatives, Eigen::Matrix<double, States + M + N, 1> y,
    double t_inverse) {
  Eigen::Matrix<double, States + M + N, 1> result;

  Eigen::Ref<Eigen::Matrix<double, States, 1>> r_dual =
      result.template block<States, 1>(0, 0);
  Eigen::Ref<Eigen::Matrix<double, M, 1>> r_cent =
      result.template block<M, 1>(States, 0);
  Eigen::Ref<Eigen::Matrix<double, N, 1>> r_pri =
      result.template block<N, 1>(States + M, 0);

  Eigen::Ref<const Eigen::Matrix<double, M, 1>> lambda =
      y.template block<M, 1>(States, 0);
  Eigen::Ref<const Eigen::Matrix<double, N, 1>> v =
      y.template block<N, 1>(States + M, 0);

  r_dual = derivatives.gradient + derivatives.df.transpose() * lambda +
           derivatives.A.transpose() * v;
  r_cent = -lambda.array() * derivatives.f.array() - t_inverse;
  r_pri = derivatives.Axmb;

  return result;
}

template <size_t States, size_t M, size_t N>
Eigen::Matrix<double, States, 1> Solver<States, M, N>::Solve(
    const ConvexProblem<States, M, N> &problem,
    Eigen::Ref<const Eigen::Matrix<double, States, 1>> X_initial) {
  const Eigen::IOFormat kHeavyFormat(Eigen::StreamPrecision, 0, ", ",
                                     ",\n                        "
                                     "                                     ",
                                     "[", "]", "[", "]");

  Eigen::Matrix<double, States + M + N, 1> y =
      Eigen::Matrix<double, States + M + N, 1>::Constant(1.0);
  y.template block<States, 1>(0, 0) = X_initial;

  Derivatives derivatives = ComputeDerivative(problem, y);

  for (size_t i = 0; i < M; ++i) {
    CHECK_LE(derivatives.f(i, 0), 0.0)
        << ": Initial state " << X_initial.transpose().format(kHeavyFormat)
        << " not feasible";
  }

  PrintDerivatives(derivatives, y, "", 1);

  size_t iteration = 0;
  while (true) {
    // Solve for the primal-dual search direction by solving the newton step.
    Eigen::Ref<const Eigen::Matrix<double, M, 1>> lambda =
        y.template block<M, 1>(States, 0);

    const double nu = -(derivatives.f.transpose() * lambda)(0, 0);
    const double t_inverse = nu / (kMu * lambda.rows());
    Eigen::Matrix<double, States + M + N, 1> rt_orig =
        Rt(derivatives, y, t_inverse);

    Eigen::Matrix<double, States + M + N, States + M + N> m1;
    m1.setZero();
    m1.template block<States, States>(0, 0) = derivatives.hessian;
    m1.template block<States, M>(0, States) = derivatives.df.transpose();
    m1.template block<States, N>(0, States + M) = derivatives.A.transpose();
    m1.template block<M, States>(States, 0) =
        -(Eigen::DiagonalMatrix<double, M>(lambda) * derivatives.df);
    m1.template block<M, M>(States, States) =
        Eigen::DiagonalMatrix<double, M>(-derivatives.f);
    m1.template block<N, States>(States + M, 0) = derivatives.A;

    Eigen::Matrix<double, States + M + N, 1> dy =
        m1.colPivHouseholderQr().solve(-rt_orig);

    Eigen::Ref<Eigen::Matrix<double, M, 1>> dlambda =
        dy.template block<M, 1>(States, 0);

    double s = 1.0;

    // Now, time to do line search.
    //
    // Start by keeping lambda positive.  Make sure our step doesn't let
    // lambda cross 0.
    for (int i = 0; i < dlambda.rows(); ++i) {
      if (lambda(i) + s * dlambda(i) < 0.0) {
        // Ignore tiny steps in lambda.  They cause issues when we get really
        // close to having our constraints met but haven't converged the rest
        // of the problem and start to run into rounding issues in the matrix
        // solve portion.
        if (dlambda(i) < 0.0 && dlambda(i) > -1e-12) {
          VLOG(1) << "  lambda(" << i << ") " << lambda(i) << " + " << s
                  << " * " << dlambda(i) << " -> s would be now "
                  << -lambda(i) / dlambda(i);
          dlambda(i) = 0.0;
          VLOG(1) << "  dy -> " << std::setprecision(12) << std::fixed
                  << std::setfill(' ') << dy.transpose().format(kHeavyFormat);
          continue;
        }
        VLOG(1) << "  lambda(" << i << ") " << lambda(i) << " + " << s << " * "
                << dlambda(i) << " -> s now " << -lambda(i) / dlambda(i);
        s = -lambda(i) / dlambda(i);
      }
    }

    VLOG(1) << "  After lambda line search, s is " << s;

    VLOG(3) << "  Initial step " << iteration << " -> " << std::setprecision(12)
            << std::fixed << std::setfill(' ')
            << dy.transpose().format(kHeavyFormat);
    VLOG(3) << "   rt ->                                        "
            << std::setprecision(12) << std::fixed << std::setfill(' ')
            << rt_orig.transpose().format(kHeavyFormat);

    const double rt_orig_squared_norm = rt_orig.squaredNorm();

    Eigen::Matrix<double, States + M + N, 1> next_y;
    Eigen::Matrix<double, States + M + N, 1> rt;
    Derivatives next_derivatives;
    while (true) {
      next_y = y + s * dy;
      next_derivatives = ComputeDerivative(problem, next_y);
      rt = Rt(next_derivatives, next_y, t_inverse);

      const Eigen::Ref<const Eigen::VectorXd> next_x =
          next_y.block(0, 0, next_derivatives.hessian.rows(), 1);
      const Eigen::Ref<const Eigen::VectorXd> next_lambda =
          next_y.block(next_x.rows(), 0, next_derivatives.f.rows(), 1);

      const Eigen::Ref<const Eigen::VectorXd> next_v = next_y.block(
          next_x.rows() + next_lambda.rows(), 0, next_derivatives.A.rows(), 1);

      VLOG(1) << "    next_rt(" << iteration << ") is " << rt.norm() << " -> "
              << std::setprecision(12) << std::fixed << std::setfill(' ')
              << rt.transpose().format(kHeavyFormat);

      PrintDerivatives(next_derivatives, next_y, "next_", 3);

      if (next_derivatives.f.maxCoeff() > 0.0) {
        VLOG(1) << "   f_next > 0.0  -> " << next_derivatives.f.maxCoeff()
                << ", continuing line search.";
        s *= kBeta;
      } else if (next_derivatives.Axmb.squaredNorm() < 0.1 &&
                 rt.squaredNorm() >
                     std::pow(1.0 - kAlpha * s, 2.0) * rt_orig_squared_norm) {
        VLOG(1) << "   |Rt| > |Rt+1| " << rt.norm() << " >  " << rt_orig.norm()
                << ", drt -> " << std::setprecision(12) << std::fixed
                << std::setfill(' ')
                << (rt_orig - rt).transpose().format(kHeavyFormat);
        s *= kBeta;
      } else {
        break;
      }
    }

    VLOG(1) << "  Terminated line search with s " << s << ", " << rt.norm()
            << "(|Rt+1|) < " << rt_orig.norm() << "(|Rt|)";
    y = next_y;

    const Eigen::Ref<const Eigen::VectorXd> next_lambda =
        y.template block<M, 1>(States, 0);

    // See if we hit our convergence criteria.
    const double r_primal_squared_norm =
        rt.template block<N, 1>(States + M, 0).squaredNorm();
    VLOG(1) << "  rt_next(" << iteration << ") is " << rt.norm() << " -> "
            << std::setprecision(12) << std::fixed << std::setfill(' ')
            << rt.transpose().format(kHeavyFormat);
    if (r_primal_squared_norm < kEpsilonF * kEpsilonF) {
      const double r_dual_squared_norm =
          rt.template block<States, 1>(0, 0).squaredNorm();
      if (r_dual_squared_norm < kEpsilonF * kEpsilonF) {
        const double next_nu =
            -(next_derivatives.f.transpose() * next_lambda)(0, 0);
        if (next_nu < kEpsilon) {
          VLOG(1) << "  r_primal(" << iteration << ") -> "
                  << std::sqrt(r_primal_squared_norm) << " < " << kEpsilonF
                  << ", r_dual(" << iteration << ") -> "
                  << std::sqrt(r_dual_squared_norm) << " < " << kEpsilonF
                  << ", nu(" << iteration << ") -> " << next_nu << " < "
                  << kEpsilon;
          break;
        } else {
          VLOG(1) << "  nu(" << iteration << ") -> " << next_nu << " < "
                  << kEpsilon << ", not done yet";
        }

      } else {
        VLOG(1) << "  r_dual(" << iteration << ") -> "
                << std::sqrt(r_dual_squared_norm) << " < " << kEpsilonF
                << ", not done yet";
      }
    } else {
      VLOG(1) << "  r_primal(" << iteration << ") -> "
              << std::sqrt(r_primal_squared_norm) << " < " << kEpsilonF
              << ", not done yet";
    }
    VLOG(1) << "  step(" << iteration << ") " << std::setprecision(12)
            << (s * dy).transpose().format(kHeavyFormat);
    VLOG(1) << " y(" << iteration << ") is now " << std::setprecision(12)
            << y.transpose().format(kHeavyFormat);

    // Very import, use the last set of derivatives we picked for our new y
    // for the next iteration.  This avoids re-computing it.
    derivatives = std::move(next_derivatives);

    ++iteration;
    if (iteration > 100) {
      LOG(FATAL) << "Too many iterations";
    }
  }

  return y.template block<States, 1>(0, 0);
}

template <size_t States, size_t M, size_t N>
typename Solver<States, M, N>::Derivatives
Solver<States, M, N>::ComputeDerivative(
    const ConvexProblem<States, M, N> &problem,
    const Eigen::Ref<const Eigen::Matrix<double, States + M + N, 1>> y) {
  const Eigen::Ref<const Eigen::Matrix<double, States, 1>> x =
      y.template block<States, 1>(0, 0);

  Derivatives derivatives;
  derivatives.gradient = problem.df0(x);
  derivatives.hessian = problem.ddf0(x);
  derivatives.f = problem.f(x);
  derivatives.df = problem.df(x);
  derivatives.A = problem.A();
  derivatives.Axmb =
      derivatives.A * y.template block<States, 1>(0, 0) - problem.b();
  return derivatives;
}

template <size_t States, size_t M, size_t N>
void Solver<States, M, N>::PrintDerivatives(
    const Derivatives &derivatives,
    const Eigen::Ref<const Eigen::Matrix<double, States + M + N, 1>> y,
    std::string_view prefix, int verbosity) {
  const Eigen::Ref<const Eigen::VectorXd> x =
      y.block(0, 0, derivatives.hessian.rows(), 1);
  const Eigen::Ref<const Eigen::VectorXd> lambda =
      y.block(x.rows(), 0, derivatives.f.rows(), 1);

  if (VLOG_IS_ON(verbosity)) {
    Eigen::IOFormat heavy(Eigen::StreamPrecision, 0, ", ",
                          ",\n                        "
                          "                                     ",
                          "[", "]", "[", "]");
    heavy.rowSeparator =
        heavy.rowSeparator +
        std::string(absl::StrCat(getpid()).size() + prefix.size(), ' ');

    const Eigen::Ref<const Eigen::VectorXd> v =
        y.block(x.rows() + lambda.rows(), 0, derivatives.A.rows(), 1);
    VLOG(verbosity) << "   " << prefix << "x: " << x.transpose().format(heavy);
    VLOG(verbosity) << "   " << prefix
                    << "lambda: " << lambda.transpose().format(heavy);
    VLOG(verbosity) << "   " << prefix << "v: " << v.transpose().format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "hessian:     " << derivatives.hessian.format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "gradient:    " << derivatives.gradient.format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "A:           " << derivatives.A.format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "Ax-b:        " << derivatives.Axmb.format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "f:           " << derivatives.f.format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "df:          " << derivatives.df.format(heavy);
  }
}

};  // namespace solvers
};  // namespace frc971

#endif  // FRC971_SOLVERS_CONVEX_H_
