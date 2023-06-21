#include "frc971/solvers/sparse_convex.h"

#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "absl/strings/str_join.h"
#include "glog/logging.h"

namespace frc971 {
namespace solvers {

Eigen::VectorXd SparseSolver::Rt(const Derivatives &derivatives,
                                 Eigen::VectorXd y, double t_inverse) {
  Eigen::VectorXd result(derivatives.states() +
                         derivatives.inequality_constraints() +
                         derivatives.equality_constraints());

  // states x 1
  Eigen::Ref<Eigen::VectorXd> r_dual =
      result.block(0, 0, derivatives.states(), 1);
  // inequality_constraints x 1
  Eigen::Ref<Eigen::VectorXd> r_cent = result.block(
      derivatives.states(), 0, derivatives.inequality_constraints(), 1);
  // equality_constraints x 1
  Eigen::Ref<Eigen::VectorXd> r_pri =
      result.block(derivatives.states() + derivatives.inequality_constraints(),
                   0, derivatives.equality_constraints(), 1);

  // inequality_constraints x 1
  Eigen::Ref<const Eigen::VectorXd> lambda =
      y.block(derivatives.states(), 0, derivatives.inequality_constraints(), 1);
  // equality_constraints x 1
  Eigen::Ref<const Eigen::VectorXd> v =
      y.block(derivatives.states() + derivatives.inequality_constraints(), 0,
              derivatives.equality_constraints(), 1);

  r_dual = derivatives.gradient + derivatives.df.transpose() * lambda +
           derivatives.A.transpose() * v;
  r_cent = -lambda.array() * derivatives.f.array() - t_inverse;
  r_pri = derivatives.Axmb;

  return result;
}

void AppendColumns(std::vector<Eigen::Triplet<double>> *triplet_list,
                   size_t starting_row, size_t starting_column,
                   const Eigen::SparseMatrix<double> &matrix) {
  for (int k = 0; k < matrix.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(matrix,
                                                                        k);
         it; ++it) {
      (*triplet_list)
          .emplace_back(it.row() + starting_row, it.col() + starting_column,
                        it.value());
    }
  }
}

void AppendColumns(
    std::vector<Eigen::Triplet<double>> *triplet_list, size_t starting_row,
    size_t starting_column,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic> &matrix) {
  for (int k = 0; k < matrix.rows(); ++k) {
    (*triplet_list)
        .emplace_back(k + starting_row, k + starting_column,
                      matrix.diagonal()(k));
  }
}

Eigen::VectorXd SparseSolver::Solve(
    const SparseConvexProblem &problem,
    Eigen::Ref<const Eigen::VectorXd> X_initial) {
  CHECK_EQ(static_cast<size_t>(X_initial.rows()), problem.states());
  CHECK_EQ(X_initial.cols(), 1);

  const Eigen::IOFormat kHeavyFormat(Eigen::StreamPrecision, 0, ", ",
                                     ",\n                        "
                                     "                                     ",
                                     "[", "]", "[", "]");

  Eigen::VectorXd y = Eigen::VectorXd::Constant(
      problem.states() + problem.inequality_constraints() +
          problem.equality_constraints(),
      1.0);
  y.block(0, 0, problem.states(), 1) = X_initial;

  Derivatives derivatives = ComputeDerivative(problem, y);

  for (size_t i = 0; i < problem.inequality_constraints(); ++i) {
    CHECK_LE(derivatives.f(i, 0), 0.0)
        << ": Initial state " << X_initial.transpose().format(kHeavyFormat)
        << " not feasible";
  }

  PrintDerivatives(derivatives, y, "", 1);

  size_t iteration = 0;
  while (true) {
    // Solve for the primal-dual search direction by solving the newton step.

    // inequality_constraints x 1;
    Eigen::Ref<const Eigen::VectorXd> lambda =
        y.block(problem.states(), 0, problem.inequality_constraints(), 1);

    const double nu = -(derivatives.f.transpose() * lambda)(0, 0);
    const double t_inverse = nu / (kMu * lambda.rows());
    Eigen::VectorXd rt_orig = Rt(derivatives, y, t_inverse);

    std::vector<Eigen::Triplet<double>> triplet_list;

    AppendColumns(&triplet_list, 0, 0, derivatives.hessian);
    AppendColumns(&triplet_list, 0, problem.states(),
                  derivatives.df.transpose());
    AppendColumns(&triplet_list, 0,
                  problem.states() + problem.inequality_constraints(),
                  derivatives.A.transpose());

    // TODO(austin): I think I can do better on the next 2, making them more
    // efficient and not creating the intermediate matrix.
    AppendColumns(&triplet_list, problem.states(), 0,
                  -(Eigen::DiagonalMatrix<double, Eigen::Dynamic>(lambda) *
                    derivatives.df));
    AppendColumns(
        &triplet_list, problem.states(), problem.states(),
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(-derivatives.f));

    AppendColumns(&triplet_list,
                  problem.states() + problem.inequality_constraints(), 0,
                  derivatives.A);

    Eigen::SparseMatrix<double> m1(
        problem.states() + problem.inequality_constraints() +
            problem.equality_constraints(),
        problem.states() + problem.inequality_constraints() +
            problem.equality_constraints());
    m1.setFromTriplets(triplet_list.begin(), triplet_list.end());

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.analyzePattern(m1);
    solver.factorize(m1);
    Eigen::VectorXd dy = solver.solve(-rt_orig);

    Eigen::Ref<Eigen::VectorXd> dlambda =
        dy.block(problem.states(), 0, problem.inequality_constraints(), 1);

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

    Eigen::VectorXd next_y;
    Eigen::VectorXd rt;
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
        y.block(problem.states(), 0, problem.inequality_constraints(), 1);

    // See if we hit our convergence criteria.
    const double r_primal_squared_norm =
        rt.block(problem.states() + problem.inequality_constraints(), 0,
                 problem.equality_constraints(), 1)
            .squaredNorm();
    VLOG(1) << "  rt_next(" << iteration << ") is " << rt.norm() << " -> "
            << std::setprecision(12) << std::fixed << std::setfill(' ')
            << rt.transpose().format(kHeavyFormat);
    if (r_primal_squared_norm < kEpsilonF * kEpsilonF) {
      const double r_dual_squared_norm =
          rt.block(0, 0, problem.states(), 1).squaredNorm();
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

  return y.block(0, 0, problem.states(), 1);
}

SparseSolver::Derivatives SparseSolver::ComputeDerivative(
    const SparseConvexProblem &problem,
    const Eigen::Ref<const Eigen::VectorXd> y) {
  // states x 1
  const Eigen::Ref<const Eigen::VectorXd> x =
      y.block(0, 0, problem.states(), 1);

  Derivatives derivatives;
  derivatives.gradient = problem.df0(x);
  CHECK_EQ(static_cast<size_t>(derivatives.gradient.rows()), problem.states());
  CHECK_EQ(static_cast<size_t>(derivatives.gradient.cols()), 1u);

  derivatives.hessian = problem.ddf0(x);
  CHECK_EQ(static_cast<size_t>(derivatives.hessian.rows()), problem.states());
  CHECK_EQ(static_cast<size_t>(derivatives.hessian.cols()), problem.states());

  derivatives.f = problem.f(x);
  CHECK_EQ(static_cast<size_t>(derivatives.f.rows()),
           problem.inequality_constraints());
  CHECK_EQ(static_cast<size_t>(derivatives.f.cols()), 1u);

  derivatives.df = problem.df(x);
  CHECK_EQ(static_cast<size_t>(derivatives.df.rows()),
           problem.inequality_constraints());
  CHECK_EQ(static_cast<size_t>(derivatives.df.cols()), problem.states());

  derivatives.A = problem.A();
  CHECK_EQ(static_cast<size_t>(derivatives.A.rows()),
           problem.equality_constraints());
  CHECK_EQ(static_cast<size_t>(derivatives.A.cols()), problem.states());

  derivatives.Axmb =
      derivatives.A * y.block(0, 0, problem.states(), 1) - problem.b();
  CHECK_EQ(static_cast<size_t>(derivatives.Axmb.rows()),
           problem.equality_constraints());
  CHECK_EQ(static_cast<size_t>(derivatives.Axmb.cols()), 1u);

  return derivatives;
}

void SparseSolver::PrintDerivatives(const Derivatives &derivatives,
                                    const Eigen::Ref<const Eigen::VectorXd> y,
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
    VLOG(verbosity) << "  " << prefix << "hessian:     " << derivatives.hessian;
    VLOG(verbosity) << "  " << prefix
                    << "gradient:    " << derivatives.gradient;
    VLOG(verbosity) << "  " << prefix << "A:           " << derivatives.A;
    VLOG(verbosity) << "  " << prefix
                    << "Ax-b:        " << derivatives.Axmb.format(heavy);
    VLOG(verbosity) << "  " << prefix
                    << "f:           " << derivatives.f.format(heavy);
    VLOG(verbosity) << "  " << prefix << "df:          " << derivatives.df;
  }
}

}  // namespace solvers
}  // namespace frc971
