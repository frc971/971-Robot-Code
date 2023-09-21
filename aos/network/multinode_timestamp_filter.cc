#include "aos/network/multinode_timestamp_filter.h"

#include <chrono>
#include <functional>
#include <map>

#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"

DEFINE_bool(timestamps_to_csv, false,
            "If true, write all the time synchronization information to a set "
            "of CSV files in /tmp/.  This should only be needed when debugging "
            "time synchronization.");

DEFINE_string(timestamp_csv_folder, "/tmp", "Folder to drop CSVs in");

DEFINE_int32(max_invalid_distance_ns, 0,
             "The max amount of time we will let the solver go backwards.");

DEFINE_int32(debug_solve_number, -1,
             "If nonzero, print out all the state for the provided solve "
             "number.  This is typically used by solving once, taking note of "
             "which solution failed to converge, and then re-running with "
             "debug turned on for just that problem.");

DEFINE_bool(bounds_offset_error, false,
            "If true, use the offset to the bounds for solving instead of to "
            "the interpolation lines.  This seems to make startup a bit "
            "better, but won't track the middle as well.");

DEFINE_bool(
    crash_on_solve_failure, true,
    "If true, crash when the solver fails to converge.  If false, keep going.  "
    "This should only be set to false when trying to see if future problems "
    "would be solvable.  This won't process a valid log");

DEFINE_bool(attempt_simultaneous_constrained_solve, true,
            "If true, try the simultaneous, constrained solver.  If false, "
            "only solve constrained problems sequentially.");

DEFINE_bool(
    remove_unlikely_constraints, true,
    "If true, when solving, try with our best guess at which constraints will "
    "be relevant and resolve if that proves wrong with an updated set.  For "
    "expensive problems, this reduces solve time significantly.");

DEFINE_int32(solve_verbosity, 1, "Verbosity to use when debugging the solver.");

DEFINE_bool(constrained_solve, true,
            "If true, use the constrained solver.  If false, only solve "
            "unconstrained.");

#define SOLVE_VLOG_IS_ON(solve_number, v)                             \
  (VLOG_IS_ON(v) ||                                                   \
   (static_cast<int32_t>(solve_number) == FLAGS_debug_solve_number && \
    v <= FLAGS_solve_verbosity))

#define SOLVE_VLOG(solve_number, v) \
  LOG_IF(INFO, SOLVE_VLOG_IS_ON(solve_number, v))

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;
using aos::logger::BootDuration;
using aos::logger::BootTimestamp;

const Eigen::IOFormat kHeavyFormat(Eigen::StreamPrecision, 0, ", ",
                                   ",\n                                        "
                                   "                                     ",
                                   "[", "]", "[", "]");

template <class... Args>
std::string CsvPath(Args &&...args) {
  return absl::StrCat(FLAGS_timestamp_csv_folder, "/",
                      std::forward<Args>(args)...);
}
}  // namespace

size_t NewtonSolver::solve_number_ = 0u;

NewtonSolver::NewtonSolver() : my_solve_number_(solve_number_++) {}

TimestampProblem::TimestampProblem(size_t count) {
  CHECK_GT(count, 1u);
  clock_offset_filter_for_node_.resize(count);
  base_clock_.resize(count);
  live_.resize(count, true);
  node_mapping_.resize(count, 0);
}

// TODO(austin): Add a rate of change constraint from the last sample.  1
// ms/s.  Figure out how to define it.  Do this last.  This lets us handle
// constraints going away, and constraints close in time.

bool TimestampProblem::HasObservations(size_t node_a) const {
  // Note, this function is probably over conservative.  It is requiring all the
  // pairs for a node to have data in at least one direction rather than enough
  // pairs to have data to observe the graph.  We can break that when someone
  // finds it is overly restrictive.

  if (clock_offset_filter_for_node_[node_a].empty()) {
    // Look for a filter going the other way who's node_b is our node.
    bool found_filter = false;
    for (size_t node_b = 0u; node_b < clock_offset_filter_for_node_.size();
         ++node_b) {
      for (const struct FilterPair &filter :
           clock_offset_filter_for_node_[node_b]) {
        if (filter.b_index == node_a) {
          if (filter.filter->timestamps_empty(base_clock_[node_b].boot,
                                              base_clock_[node_a].boot)) {
            // Found one without data, explode.
            return false;
          }
          found_filter = true;
        }
      }
    }
    return found_filter;
  }

  for (const struct FilterPair &filter :
       clock_offset_filter_for_node_[node_a]) {
    // There's something in this direction, so we don't need to check the
    // opposite direction to confirm we have observations.
    if (!filter.filter->timestamps_empty(base_clock_[node_a].boot,
                                         base_clock_[filter.b_index].boot)) {
      continue;
    }

    // For a boot to exist, we need to have some observations between it and
    // another boot.  We wouldn't bother to build a problem to solve for
    // this node otherwise.  Confirm that is true so we at least get
    // notified if that assumption falls apart.
    if (filter.b_filter == nullptr) {
      return false;
    }
    return !filter.b_filter->timestamps_empty(base_clock_[filter.b_index].boot,
                                              base_clock_[node_a].boot);
  }
  return true;
}

bool TimestampProblem::ValidateSolution(std::vector<BootTimestamp> solution,
                                        bool quiet) {
  bool success = true;
  for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
    for (const struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
      // There's nothing in this direction, so there will be nothing to
      // validate.
      if (filter.filter->timestamps_empty(base_clock_[i].boot,
                                          base_clock_[filter.b_index].boot)) {
        // For a boot to exist, we need to have some observations between it and
        // another boot.  We wouldn't bother to build a problem to solve for
        // this node otherwise.  Confirm that is true so we at least get
        // notified if that assumption falls apart.
        if (filter.b_filter == nullptr ||
            filter.b_filter->timestamps_empty(base_clock_[filter.b_index].boot,
                                              base_clock_[i].boot)) {
          Debug();
          LOG(FATAL) << "Found no timestamps in either direction between nodes "
                     << i << " and " << filter.b_index;
        }
        continue;
      }
      const bool iteration = filter.filter->ValidateSolution(
          filter.b_filter, filter.pointer, solution[i],
          solution[filter.b_index], true, quiet);
      if (!iteration) {
        filter.filter->ValidateSolution(
            filter.b_filter, filter.pointer, solution[i], 0.0,
            solution[filter.b_index], 0.0, true, quiet);
      }

      success = success && iteration;
    }
  }
  return success;
}

size_t TimestampProblem::LiveConstraintsCount() const {
  size_t result = 0;
  for (size_t i = 0; i < clock_offset_filter_for_node_.size(); ++i) {
    for (const struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
      // Especially when reboots are involved, it isn't guarenteed that there
      // will be timestamps going both ways.  In this case, we want to avoid the
      // cost.
      if (filter.filter->timestamps_empty(base_clock_[i].boot,
                                          base_clock_[filter.b_index].boot)) {
        continue;
      }
      ++result;
    }
  }
  return result;
}

Problem::Derivatives TimestampProblem::ComputeDerivatives(
    size_t solve_number, const Eigen::Ref<const Eigen::VectorXd> time_offsets,
    bool quiet, bool all, const absl::Span<const size_t> active_constraints) {
  // Queue long explanation for why this is the right math...
  //
  // Our cost function is piecewise quadratic and simple by design.  It should
  // also be almost convex.  This means it is equivalent for us to drive the
  // gradient to 0 to find the corresponding times on all nodes.
  //
  // This gets us close but doesn't let us solve for the time corresponding to a
  // specific time on one node on all the nodes.
  //
  // To do this, we want a Newton solver which works for equality constraints.
  //   argmin f(x)
  //   subject to {A x = b}
  //
  // More specifically, we want a version of this which will start with
  // infeasible initial solutions.
  //
  // The newton step for this is
  //   X(n+1) = X(n) + xnt
  //
  //   [Hessian(cost(X(n))) A^T] [xnt] = [-grad(cost(X(n)))]
  //   [A                     0] [w]     [-A X(n) + b      ]
  //
  // It turns out that w is the dual newton step.  But we don't actually need to
  // track the dual problem to solve our problem except to show that the dual
  // problem has also converged.
  //
  // We could set A to [1, 0, 0, ...] and force a single clock to a specific
  // time.  But, that will result in the optimal solution being different
  // depending on which node is picked.
  //
  // Instead, let's solve for the average clock instead.  This will be always
  // symmetric, and we can drive the goal for that clock to make any individual
  // clock have the right time.  That would be a solver wrapped around a solver.
  //
  // Turns out, we can do that by combining the iterations.  If we set A to
  // [1/n, 1/n, 1/n ...], and b to the distributed clock, that would drive our
  // states such that the distributed clock will be what we want.  If we instead
  // leave A the same, but set (-A X(n) + b) to be ( - [1, 0, 0...] * X(n) +
  // goal_clock), we will drive the distributed clock to be what it needs to be
  // to set a node's clock to the right time.
  //
  // This ends up working surprisingly well.  A toy problem with 2 line segments
  // and 2 nodes converges in 2 iterations.
  //
  // To ensure reliable convergence, we want to make 1 adjustment to the above
  // problem statement.
  //
  // d cost/dta =>
  //   2 * (tb - (1 + ma) ta - ba) * (-(1 + ma))
  //
  // This means that as you move between line segments with different slopes,
  // you end up with step changes in the gradient.  Solvers like continuous
  // derivatives.  But, we don't really care if this is an exact solution to the
  // cost problem.  We just care that it is close to a solution to the cost
  // problem and more importantly well behaved.
  //
  // The simple fix is to ignore the slope when applying the chain rule.  This
  // makes the derivative just be the distance to the line, which is a
  // continuous function.  Newtons method then converges really really easily
  // every time.
  CHECK_GT(states(), 0u) << ": No live nodes to solve for.";
  Problem::Derivatives result;

  // We get back both interger and double remainders for the gradient.  We then
  // add them all up.  Rather than doing that purely as doubles, let's save up
  // both, compute the result, then convert the remainder to doubles.  This is a
  // bigger issue at the start when we are extrapolating a lot, and the offsets
  // can be quite large in each direction.
  Eigen::Matrix<chrono::nanoseconds, Eigen::Dynamic, 1> intgrad =
      Eigen::Matrix<chrono::nanoseconds, Eigen::Dynamic, 1>::Zero(states());
  result.gradient = Eigen::VectorXd::Zero(states());

  result.hessian = Eigen::MatrixXd::Zero(states(), states());

  // Constrain the average of the times.
  result.A = Eigen::MatrixXd::Ones(1, states()) / static_cast<double>(states());
  result.Axmb = Eigen::VectorXd::Zero(1);

  if (active_constraints.size() > 0 || all) {
    const size_t constraints =
        all ? inequality_constraints_ : active_constraints.size();
    result.f = Eigen::MatrixXd::Zero(constraints, 1);
    result.df = Eigen::MatrixXd::Zero(constraints, states());
    result.df_slope_limited = Eigen::MatrixXd::Zero(constraints, states());
  }

  size_t constraint_index = 0;
  size_t result_constraint_index = 0;

  for (size_t i = 0; i < clock_offset_filter_for_node_.size(); ++i) {
    for (struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
      // Especially when reboots are involved, it isn't guarenteed that there
      // will be timestamps going both ways.  In this case, we want to avoid the
      // cost.
      if (filter.filter->timestamps_empty(base_clock_[i].boot,
                                          base_clock_[filter.b_index].boot)) {
        continue;
      }

      // Reminder, our cost function has the following form.
      //   ((tb - (1 + ma) ta - ba)^2
      // We are ignoring the slope when taking the derivative and applying the
      // chain rule to keep the gradient smooth.  This means that the gradient
      // is +- 2 * error.
      //
      const size_t a_solution_index = NodeToFullSolutionIndex(i);
      const size_t b_solution_index = NodeToFullSolutionIndex(filter.b_index);

      // Most of the time, we properly converge to the right answer when only
      // one of the two constraints exists.  But, with rounding involved, we
      // will end up generating 2 timelines with 2 problems:
      //  1) If the TX and RX times are identical (zero network delay), then it
      //     is very hard to order the two messages without adding something on
      //     top of time.
      //  2) In the presence of rounding, we can violate our constraints by 1
      //     ns, failing validation.
      //
      // Rather than teach the solver about these constraints, we can instead
      // have it try to solve for an offset which is a bit bigger than it is
      // supposed to be.  Since both directions, when they exist, will have this
      // extra factor, the solution will be the same (or close enough).
      constexpr double kMinNetworkDelay = 2.0;

      std::pair<NoncausalTimestampFilter::Pointer,
                std::tuple<chrono::nanoseconds, double, double>>
          offset_error =
              FLAGS_bounds_offset_error
                  ? filter.filter->BoundsOffsetError(
                        filter.b_filter, std::move(filter.pointer),
                        base_clock_[i], time_offsets(a_solution_index),
                        base_clock_[filter.b_index],
                        time_offsets(b_solution_index))
                  : filter.filter->OffsetError(filter.b_filter, filter.pointer,
                                               base_clock_[i],
                                               time_offsets(a_solution_index),
                                               base_clock_[filter.b_index],
                                               time_offsets(b_solution_index));

      std::pair<NoncausalTimestampFilter::Pointer,
                std::tuple<chrono::nanoseconds, double, double>>
          bounds_offset_error = filter.filter->BoundsOffsetError(
              filter.b_filter, std::move(offset_error.first), base_clock_[i],
              time_offsets(a_solution_index), base_clock_[filter.b_index],
              time_offsets(b_solution_index));

      filter.pointer = std::move(bounds_offset_error.first);

      const std::pair<chrono::nanoseconds, double> error =
          std::make_pair(std::get<0>(offset_error.second),
                         std::get<1>(offset_error.second) - kMinNetworkDelay);

      const double bounds_error =
          (std::get<0>(bounds_offset_error.second).count() +
           std::get<1>(bounds_offset_error.second) - kMinNetworkDelay);

      // We need f(X) <= 0 to be our constraint.  But, remember, OffsetError is:
      //   (tb - ta) - O(ta)
      // With a bit of reshuffling, we quickly see that for an invalid offset
      // (see ValidateSolution for what is invalid), offset + ta >= tb, we end
      // up with a negative value of f(X).  So, flip the sign to match the
      // conventions.
      if (all ||
          (result_constraint_index < active_constraints.size() &&
           active_constraints[result_constraint_index] == constraint_index)) {
        result.f(result_constraint_index, 0) = -bounds_error;

        result.df(result_constraint_index, a_solution_index) +=
            1.0 + std::get<2>(bounds_offset_error.second);
        result.df(result_constraint_index, b_solution_index) -= 1.0;

        // Now, return a slope limited version of our constraint derivitive.
        result.df_slope_limited(result_constraint_index, a_solution_index) +=
            1.0 + kMaxVelocity();
        result.df_slope_limited(result_constraint_index, b_solution_index) -=
            1.0;
        ++result_constraint_index;
      }

      // TODO(austin): A loss function here would be nice to let us deweight
      // outliers more.  Something logarithmic.
      const std::pair<chrono::nanoseconds, double> grad =
          std::make_pair(2 * error.first, 2 * error.second);
      const double hess = 2.0;

      intgrad(a_solution_index) += -grad.first;
      intgrad(b_solution_index) += grad.first;
      result.gradient(a_solution_index) += -grad.second;
      result.gradient(b_solution_index) += grad.second;

      if (!quiet) {
        SOLVE_VLOG(solve_number, 3)
            << "  Filter pair "
            << filter.filter->node_a()->name()->string_view() << "("
            << a_solution_index << ") -> "
            << filter.filter->node_b()->name()->string_view() << "("
            << b_solution_index << "): " << std::setprecision(12)
            << error.first.count() << " + " << error.second;
      }

      // Reminder, our cost function has the following form.
      //   ((tb - (1 + ma) ta - ba)^2
      // We are ignoring the slope when taking the derivative and applying the
      // chain rule to keep the gradient smooth.  This means that the Hessian
      // is 2 for d^2 cost/dta^2 and d^2 cost/dtb^2
      result.hessian(a_solution_index, a_solution_index) += hess;
      result.hessian(b_solution_index, a_solution_index) += -hess;
      result.hessian(a_solution_index, b_solution_index) =
          result.hessian(b_solution_index, a_solution_index);
      result.hessian(b_solution_index, b_solution_index) += hess;

      ++constraint_index;
    }
  }

  for (int i = 0; i < intgrad.rows(); ++i) {
    result.gradient(i) += static_cast<double>(intgrad(i).count());
  }

  // Now, we want to set A x - b to be -time_offset for the earliest
  // clock.
  //
  // To save ourselves a fair amount of compute, we can take the min here.  That
  // will drive us back the furthest back in time for all provided nodes without
  // having to solve N times and look for the earliest solution.
  for (size_t i = 0; i < points_.size(); ++i) {
    if (points_[i] == logger::BootTimestamp::max_time()) {
      continue;
    }

    CHECK_EQ(points_[i].boot, base_clock(i).boot);
    const double candidate_b = chrono::duration<double, std::nano>(
                                   points_[i].time - base_clock(i).time)
                                   .count() -
                               time_offsets(NodeToFullSolutionIndex(i));
    if ((result.Axmb.rows() != 0 && candidate_b < -result.Axmb(0, 0)) ||
        result.solution_node == std::numeric_limits<size_t>::max()) {
      if (!quiet) {
        SOLVE_VLOG(solve_number, 1)
            << "  Node " << i << ", desired solution time " << points_[i]
            << ", base_clock " << base_clock(i) << ", error " << candidate_b
            << " time offset " << time_offsets(NodeToFullSolutionIndex(i));
      }
      if (result.Axmb.rows() != 0) {
        result.Axmb(0, 0) = -candidate_b;
      }
      result.solution_node = i;
    }
  }

  CHECK_NE(result.solution_node, std::numeric_limits<size_t>::max())
      << ": No solution nodes, please investigate";

  return result;
}

Eigen::VectorXd NewtonSolver::Newton(const Problem::Derivatives &derivatives,
                                     size_t iteration) {
  // https://www.cs.purdue.edu/homes/jhonorio/16spring-cs52000-equality.pdf
  // https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf chapter 10 is good
  // too.

  Eigen::MatrixXd a;
  a.resize(derivatives.gradient.rows() + derivatives.A.rows(),
           derivatives.gradient.rows() + derivatives.A.rows());
  a.block(0, 0, derivatives.gradient.rows(), derivatives.gradient.rows()) =
      derivatives.hessian;
  a.block(0, derivatives.gradient.rows(), derivatives.gradient.rows(),
          derivatives.A.rows()) = derivatives.A.transpose();
  a.block(derivatives.gradient.rows(), 0, derivatives.A.rows(),
          derivatives.gradient.rows()) = derivatives.A;
  a(derivatives.gradient.rows(), derivatives.gradient.rows()) = 0.0;

  Eigen::VectorXd b = Eigen::VectorXd::Zero(a.rows());
  b.block(0, 0, derivatives.gradient.rows(), 1) = -derivatives.gradient;
  if (derivatives.Axmb.rows()) {
    b.block(derivatives.gradient.rows(), 0, derivatives.Axmb.rows(), 1) =
        -derivatives.Axmb;
  }

  VLOG(2) << "A: " << a.format(kHeavyFormat);
  VLOG(2) << "b: " << b.format(kHeavyFormat);

  Eigen::VectorXd step = a.colPivHouseholderQr().solve(b);

  if (SOLVE_VLOG_IS_ON(my_solve_number_, 2)) {
    // Print out the gradient ignoring the component removed by the equality
    // constraint.  This tells us what gradient we are depending to try to
    // finish our solution.
    const Eigen::MatrixXd constraint_jacobian =
        Eigen::MatrixXd::Ones(1, derivatives.gradient.rows()) /
        static_cast<double>(derivatives.gradient.rows());
    Eigen::VectorXd adjusted_grad =
        derivatives.gradient +
        step(derivatives.gradient.rows()) * constraint_jacobian.transpose();

    SOLVE_VLOG(my_solve_number_, 2)
        << "Adjusted grad " << iteration << " -> " << std::setprecision(12)
        << std::fixed << std::setfill(' ')
        << adjusted_grad.transpose().format(kHeavyFormat);
  }

  return step;
}

double NewtonSolver::RSquaredUnconstrained(
    const Problem::Derivatives &derivatives,
    Eigen::Ref<const Eigen::VectorXd> y, Eigen::Ref<const Eigen::VectorXd> step,
    double t) {
  // See https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf section 10.3.1
  //
  // Note: all the prints are transposed so they fit on one line instead of
  // having a ton of massive column vectors being printed all the time.
  SOLVE_VLOG(my_solve_number_, 2)
      << "    r_primal: " << std::setprecision(12) << std::fixed
      << std::setfill(' ') << derivatives.Axmb.transpose().format(kHeavyFormat);
  SOLVE_VLOG(my_solve_number_, 2)
      << "    r_dual: " << std::setprecision(12) << std::fixed
      << std::setfill(' ')
      << derivatives.gradient.transpose().format(kHeavyFormat) << " + "
      << ((y.block(derivatives.gradient.rows(), 0, 1, 1) +
           t * step.block(derivatives.gradient.rows(), 0, 1, 1))
              .transpose() *
          derivatives.A)
             .transpose()
             .format(kHeavyFormat);
  return (derivatives.gradient +
          derivatives.A.transpose() *
              (y.block(derivatives.gradient.rows(), 0, 1, 1) +
               t * step.block(derivatives.gradient.rows(), 0, 1, 1)))
             .squaredNorm() +
         derivatives.Axmb.squaredNorm();
}

std::tuple<Eigen::VectorXd, size_t, size_t> NewtonSolver::SolveNewton(
    Problem *problem, const size_t max_iterations) {
  SOLVE_VLOG(my_solve_number_, 2)
      << "Starting to solve problem number " << my_solve_number_;

  problem->Prepare(my_solve_number_);

  Eigen::VectorXd y = Eigen::VectorXd::Zero(problem->states() + 1);

  size_t iteration = 0;
  size_t solution_node;

  // The derivatives and squared norm of r(y).
  Problem::Derivatives derivatives = problem->ComputeDerivatives(
      my_solve_number_, y, false, false, absl::Span<const size_t>());
  double r_orig_squared = RSquaredUnconstrained(derivatives, y, y, 0.0);
  while (true) {
    const Eigen::VectorXd step = Newton(derivatives, iteration);
    solution_node = derivatives.solution_node;

    SOLVE_VLOG(my_solve_number_, 2)
        << "Step " << iteration << " -> " << std::setprecision(12) << std::fixed
        << std::setfill(' ') << step.transpose().format(kHeavyFormat);

    // We now have a search direction.  Line search and go.
    double t = 1.0;

    // We got there if the max step is small (this is strongly correlated to
    // the gradient since the Hessian is constant), and our solution node's
    // time is also close.

    // Grad is < epsilon.
    // Dual is < epsilon too.
    if (step.block(0, 0, problem->states(), 1).lpNorm<Eigen::Infinity>() <
            1e-4 &&
        derivatives.Axmb.squaredNorm() < 1e-8) {
      break;
    }

    // Now, do line search.
    while (true) {
      SOLVE_VLOG(my_solve_number_, 3) << "   Trying t: " << t;
      Problem::Derivatives new_derivatives =
          problem->ComputeDerivatives(my_solve_number_, y + t * step, false,
                                      false, absl::Span<const size_t>());
      const double r_squared =
          RSquaredUnconstrained(new_derivatives, y, step, t);
      // See Boyd, section 10.3.2, algorithm 10.2
      if (r_squared <= std::pow(1.0 - kAlpha * t, 2.0) * r_orig_squared) {
        // Save the derivatives and norm computed for the next iteration to save
        // CPU.
        derivatives = std::move(new_derivatives);
        r_orig_squared = r_squared;
        SOLVE_VLOG(my_solve_number_, 3)
            << "  Line search terminated with a step of " << t;
        break;
      } else {
        CHECK_NE(t, 0.0) << ": Failed on solve " << my_solve_number_;
      }
      t *= kBeta;
    }

    y += t * step;

    ++iteration;

    // We are doing all our math with both an int64 base and a double offset.
    // This lets us handle large offsets while retaining precision down to the
    // nanosecond easily.
    //
    // Some problems start out with a poor initial solution.  This is especially
    // true for the first solution.  Because we control the solver, as we
    // determine that the double is getting too big, we can move that
    // information to the int64 base clock.  Threshold this to not be *too* big
    // since it makes it hard to debug as y keeps jumping around.
    problem->Update(my_solve_number_, y);

    // And finally, don't let us iterate forever.  If it isn't converging,
    // report back.
    if (iteration > max_iterations) {
      break;
    }
  }

  if (iteration > max_iterations) {
    LOG(ERROR) << "Failed to converge on solve " << my_solve_number();
  }

  SOLVE_VLOG(my_solve_number_, 1)
      << "Took " << iteration << " iterations to solve.";
  return std::make_tuple(y.block(0, 0, problem->states(), 1), solution_node,
                         iteration);
}

Problem::Derivatives NewtonSolver::AddConstraintSlackVariable(
    const Problem::Derivatives &derivatives,
    const Eigen::Ref<const Eigen::VectorXd> y) {
  // See ConstrainedNewton() for an explanation of how we are changing the
  // problem statement.
  //
  //   minimize f0(X, s)
  //     subject to f(X) <= s
  //                 A X  = b
  //                   s  = 0
  Problem::Derivatives result;

  // There's no extra cost hessian as part of the new slack variable, since it
  // isn't in the cost function, f0.
  result.hessian = Eigen::MatrixXd::Zero(derivatives.hessian.rows() + 1,
                                         derivatives.hessian.rows() + 1);
  result.hessian.block(0, 0, derivatives.hessian.rows(),
                       derivatives.hessian.rows()) = derivatives.hessian;

  // And the gradient of that cost is also zero.
  result.gradient = Eigen::VectorXd::Zero(derivatives.gradient.rows() + 1);

  result.gradient.block(0, 0, derivatives.gradient.rows(),
                        derivatives.gradient.cols()) = derivatives.gradient;

  // Subtract the new slack variable from f(x).
  result.f = derivatives.f.array() - y(derivatives.gradient.rows(), 0);

  // And the derivitive of f(x) - s * Ones with respect to s is -1.
  result.df = -1.0 * Eigen::MatrixXd::Ones(derivatives.df.rows(),
                                           derivatives.df.cols() + 1);
  result.df.block(0, 0, derivatives.df.rows(), derivatives.df.cols()) =
      derivatives.df;

  result.df_slope_limited =
      -1.0 * Eigen::MatrixXd::Ones(derivatives.df_slope_limited.rows(),
                                   derivatives.df_slope_limited.cols() + 1);
  result.df_slope_limited.block(0, 0, derivatives.df_slope_limited.rows(),
                                derivatives.df_slope_limited.cols()) =
      derivatives.df_slope_limited;

  // And add in s = 0 into A.
  result.A = Eigen::MatrixXd::Zero(derivatives.A.rows() + 1,
                                   derivatives.hessian.rows() + 1);
  result.A.block(0, 0, derivatives.A.rows(), derivatives.hessian.rows()) =
      derivatives.A;
  result.A(derivatives.A.rows(), derivatives.hessian.rows()) = 1;

  result.Axmb = Eigen::MatrixXd::Zero(derivatives.A.rows() + 1, 1);
  result.Axmb.block(0, 0, derivatives.A.rows(), 1) = derivatives.Axmb;
  result.Axmb(derivatives.A.rows(), 0) = y(derivatives.hessian.rows());
  result.solution_node = derivatives.solution_node;

  return result;
}

Eigen::VectorXd NewtonSolver::Rt(const Problem::Derivatives &derivatives,
                                 Eigen::Ref<const Eigen::VectorXd> y,
                                 double t_inverse) {
  const Eigen::Ref<const Eigen::VectorXd> x =
      y.block(0, 0, derivatives.hessian.rows(), 1);
  const Eigen::Ref<const Eigen::VectorXd> lambda =
      y.block(derivatives.hessian.rows(), 0, derivatives.f.rows(), 1);
  const Eigen::Ref<const Eigen::VectorXd> v = y.block(
      derivatives.hessian.rows() + lambda.rows(), 0, derivatives.A.rows(), 1);

  Eigen::VectorXd result(y.rows());

  // r_dual -> \grad f_0(X) + Df(x)^T \lambda + A^T v
  //
  // We need to lie slightly about the problem statement to get it to converge.
  // We need the decent direction (in ConstrainedNewton) to use the exact
  // derivative of f(x) so we will descend following the constraints when we are
  // next to a constraint to avoid getting stuck with line search.  But, we need
  // the value it is trying to converge to (this, Rt), to not have step changes
  // across constraint line segment boundaries.  If we lie here and pick the
  // worst case slope (the slope_limited slope), then there will be no step
  // change across boundaries.
  result.block(0, 0, x.rows(), 1) =
      derivatives.gradient + derivatives.df_slope_limited.transpose() * lambda +
      derivatives.A.transpose() * v;

  SOLVE_VLOG(my_solve_number_, 3)
      << "    r_dual: " << std::setprecision(12) << std::fixed
      << std::setfill(' ')
      << derivatives.gradient.transpose().format(kHeavyFormat) << " + "
      << (derivatives.df_slope_limited.transpose() * lambda)
             .transpose()
             .format(kHeavyFormat)
      << " + " << (v.transpose() * derivatives.A).format(kHeavyFormat);

  // r_cent -> -diag(\lambda) f(x) - 1 / t * \bold{1}
  result.block(x.rows(), 0, lambda.rows(), 1) =
      -lambda.array() * derivatives.f.array() - t_inverse;

  // r_primal -> A X - b
  result.block(x.rows() + lambda.rows(), 0, v.rows(), 1) = derivatives.Axmb;

  return result;
}

std::tuple<Eigen::VectorXd, double, Eigen::VectorXd>
NewtonSolver::ConstrainedNewton(const Eigen::Ref<const Eigen::VectorXd> y,
                                const Problem::Derivatives &derivatives,
                                size_t /*iteration*/) {
  // TODO(austin): Can we take a problem directly without
  // AddConstraintSlackVariable being called on it and fill everything in?
  // That'll avoid a bunch of allocations and copies of rather sizable
  // matricies.
  //
  // I think I want to do it as another review after everything works with the
  // more obviously verifiable solution.

  const Eigen::Ref<const Eigen::VectorXd> x =
      y.block(0, 0, derivatives.hessian.rows(), 1);

  const Eigen::Ref<const Eigen::VectorXd> lambda =
      y.block(x.rows(), 0, derivatives.f.rows(), 1);

  const Eigen::Ref<const Eigen::VectorXd> v =
      y.block(x.rows() + lambda.rows(), 0, derivatives.A.rows(), 1);

  // https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf chapter 11.7 is good.
  //
  // See the unconstrained solver above for the equality and cost explination.
  //
  // We need to add constraints.  Remember, our problem is of the form:
  //
  //   minimize f0(X)
  //     subject to f(X) <= 0
  //                 A X  = b
  //
  // Unfortunately, we need a feasible X to start with.  A big part of the
  // problem is finding a feasible X.  But, we can easily augment our problem to
  // make it feasible.
  //
  //   minimize f0(X, s)
  //     subject to f(X) <= s
  //                 A X  = b
  //                   s  = 0
  //
  // s becomes part of X, and then the original solver works.  We can pick an
  // arbitrarily large S to start with, mainly max(f(X), 0).  We are feasible
  // when s == 0.
  //
  // Since our constraints are linear, we don't need the second derivitive of
  // the constraints.  The equations below remove those to save the CPU usage.
  const double nu = -(derivatives.f.transpose() * lambda)(0, 0);
  const double t_inverse = nu / (kMu * lambda.rows());

  // Now, build up the step direction matrix to solve.
  Eigen::MatrixXd m1 = Eigen::MatrixXd::Zero(y.rows(), y.rows());
  m1.block(0, 0, x.rows(), x.rows()) = derivatives.hessian;
  // TODO(austin): Slope constrained or leave it stock?
  m1.block(0, x.rows(), x.rows(), lambda.rows()) = derivatives.df.transpose();

  m1.block(0, x.rows() + lambda.rows(), x.rows(), v.rows()) =
      derivatives.A.transpose();

  m1.block(x.rows(), 0, lambda.rows(), x.rows()) =
      -(Eigen::DiagonalMatrix<double, Eigen::Dynamic>(lambda) * derivatives.df);

  m1.block(x.rows(), x.rows(), lambda.rows(), lambda.rows()) =
      Eigen::DiagonalMatrix<double, Eigen::Dynamic>(-derivatives.f);

  m1.block(x.rows() + lambda.rows(), 0, v.rows(), x.rows()) = derivatives.A;

  SOLVE_VLOG(my_solve_number_, 3)
      << "   m1:         " << m1.format(kHeavyFormat);

  Eigen::VectorXd rt = Rt(derivatives, y, t_inverse);
  SOLVE_VLOG(my_solve_number_, 2)
      << "   rt: " << rt.norm() << " " << rt.transpose().format(kHeavyFormat);

  Eigen::VectorXd dy = m1.colPivHouseholderQr().solve(-rt);
  return std::tuple<Eigen::VectorXd, double, Eigen::VectorXd>(
      std::move(dy), t_inverse, std::move(rt));
}

void NewtonSolver::PrintDerivatives(const Problem::Derivatives &derivatives,
                                    const Eigen::Ref<const Eigen::VectorXd> y,
                                    std::string_view prefix, int verbosity) {
  const Eigen::Ref<const Eigen::VectorXd> x =
      y.block(0, 0, derivatives.hessian.rows(), 1);
  const Eigen::Ref<const Eigen::VectorXd> lambda =
      y.block(x.rows(), 0, derivatives.f.rows(), 1);

  if (SOLVE_VLOG_IS_ON(my_solve_number_, verbosity)) {
    Eigen::IOFormat heavy = kHeavyFormat;
    heavy.rowSeparator =
        kHeavyFormat.rowSeparator +
        std::string(absl::StrCat(getpid()).size() + prefix.size(), ' ');

    const Eigen::Ref<const Eigen::VectorXd> v =
        y.block(x.rows() + lambda.rows(), 0, derivatives.A.rows(), 1);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "   " << prefix
        << "x: " << x.transpose().format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "   " << prefix
        << "lambda: " << lambda.transpose().format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "   " << prefix
        << "v: " << v.transpose().format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "  " << prefix
        << "hessian:     " << derivatives.hessian.format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "  " << prefix
        << "gradient:    " << derivatives.gradient.format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "  " << prefix
        << "A:           " << derivatives.A.format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "  " << prefix
        << "Ax-b:        " << derivatives.Axmb.format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "  " << prefix
        << "f:           " << derivatives.f.format(heavy);
    SOLVE_VLOG(my_solve_number_, verbosity)
        << std::setprecision(12) << "  " << prefix
        << "df:          " << derivatives.df.format(heavy);
  }
}

std::vector<BootTimestamp> TimestampProblem::PackResults(
    bool print, const Eigen::Ref<const Eigen::VectorXd> y) {
  std::vector<BootTimestamp> result(size());

  for (size_t i = 0; i < size(); ++i) {
    if (live(i)) {
      result[i].boot = base_clock(i).boot;
      result[i].time =
          base_clock(i).time + std::chrono::nanoseconds(static_cast<int64_t>(
                                   std::round(y(NodeToFullSolutionIndex(i)))));
      if (print) {
        LOG(INFO) << "    live " << points_[i] << " vs solution "
                  << result[i].time << " "
                  << (y(NodeToFullSolutionIndex(i)) -
                      std::round(y(NodeToFullSolutionIndex(i))))
                  << " (unrounded: " << y(NodeToFullSolutionIndex(i)) << ") dt "
                  << (points_[i].time -
                      (base_clock_[i].time +
                       std::chrono::nanoseconds(static_cast<int64_t>(
                           std::round(y(NodeToFullSolutionIndex(i)))))))
                         .count()
                  << "ns";
      }
    } else {
      result[i] = BootTimestamp::min_time();
      if (print) {
        LOG(INFO) << "  dead  " << result[i];
      }
    }
  }

  return result;
}

bool TimestampProblem::AddConstraints(
    size_t solve_number, const std::vector<BootTimestamp> &result,
    std::vector<size_t> *new_active_constraints) {
  bool updated = false;
  {
    // TODO(austin): Make this a function.
    size_t constraint_index = 0;
    for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
      for (const struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
        // There's nothing in this direction, so there will be nothing to
        // validate.
        if (filter.filter->timestamps_empty(base_clock_[i].boot,
                                            base_clock_[filter.b_index].boot)) {
          continue;
        }

        // See if this constraint had trouble.  Note: we don't care if it is
        // before the last solution.  If it influences the result, it will
        // change the solution and we'll need to re-solve anyways.  There's a
        // big validation that happens afterwards which will pick up if we are
        // before the start.
        const bool iteration = filter.filter->ValidateSolution(
            filter.b_filter, filter.pointer, result[i], result[filter.b_index],
            false, !SOLVE_VLOG_IS_ON(solve_number, 2));
        if (!iteration) {
          // Ok, we found a violated constraint.  Add it to the list.
          updated = true;
          CHECK(std::find(new_active_constraints->begin(),
                          new_active_constraints->end(),
                          constraint_index) == new_active_constraints->end())
              << " while solving " << solve_number << " constraint index "
              << constraint_index;

          SOLVE_VLOG(solve_number, 1)
              << " Found a violated constraint at index " << constraint_index
              << " on problem " << solve_number;
          new_active_constraints->insert(
              std::upper_bound(new_active_constraints->begin(),
                               new_active_constraints->end(), constraint_index),
              constraint_index);
        }

        ++constraint_index;
      }
    }

    CHECK_EQ(constraint_index, inequality_constraints_);
  }
  return updated;
}

std::optional<
    std::tuple<std::vector<BootTimestamp>, Eigen::VectorXd, size_t, size_t>>
SolveConstrainedNewton(NewtonSolver *solver, TimestampProblem *problem,
                       const size_t max_iterations) {
  std::vector<size_t> active_constraints;

  while (true) {
    size_t iteration;
    size_t solution_node;

    auto solver_result = solver->SolveConstrainedNewton(problem, max_iterations,
                                                        active_constraints);
    if (!solver_result.has_value()) {
      return std::nullopt;
    }

    std::tie(std::ignore, solution_node, iteration, std::ignore) =
        *solver_result;
    Eigen::VectorXd y = std::move(std::get<0>(*solver_result));
    std::vector<size_t> new_active_constraints =
        std::move(std::get<3>(*solver_result));

    std::vector<BootTimestamp> result =
        problem->PackResults(iteration > max_iterations ||
                                 SOLVE_VLOG_IS_ON(solver->my_solve_number(), 2),
                             y);

    const bool valid = !problem->AddConstraints(
        solver->my_solve_number(), result, &new_active_constraints);

    if (valid ||
        active_constraints.size() == problem->inequality_constraints()) {
      return std::make_tuple(std::move(result), std::move(y), solution_node,
                             iteration);
    } else {
      CHECK(new_active_constraints != active_constraints);
      active_constraints = std::move(new_active_constraints);
      SOLVE_VLOG(solver->my_solve_number(), 1)
          << "Constraint indices changed, solving again with more constraints.";
    }
  }
}

std::optional<std::tuple<Eigen::VectorXd, size_t, size_t, std::vector<size_t>>>
NewtonSolver::SolveConstrainedNewton(
    Problem *problem, const size_t max_iterations,
    const absl::Span<const size_t> original_active_constraints) {
  // Implement a primal-dual interior point method solver.
  //
  // https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf chapter 11.7 has a
  // good description.  It is worth having the book open when reading this code.
  SOLVE_VLOG(my_solve_number_, 2)
      << "Starting to solve problem number " << my_solve_number_;

  problem->Prepare(my_solve_number_);

  Eigen::VectorXd y;
  size_t iteration = 0;
  size_t solution_node;

  std::vector<size_t> active_constraints;

  {
    const Problem::Derivatives derivatives = AddConstraintSlackVariable(
        problem->ComputeDerivatives(my_solve_number_,
                                    Eigen::VectorXd::Zero(problem->states()),
                                    false, true, absl::Span<const size_t>()),
        Eigen::VectorXd::Zero(problem->states() + 1));

    size_t original_constraint_index = 0;
    for (size_t i = 0; i < static_cast<size_t>(derivatives.f.rows()); ++i) {
      if (original_constraint_index < original_active_constraints.size() &&
          original_active_constraints[original_constraint_index] == i) {
        SOLVE_VLOG(my_solve_number_, 1)
            << " Considering constraint " << i << " from before";
        active_constraints.emplace_back(i);
        ++original_constraint_index;
      } else if (derivatives.f(i) > 0.0 || !FLAGS_remove_unlikely_constraints) {
        SOLVE_VLOG(my_solve_number_, 1) << " Considering constraint " << i;
        active_constraints.emplace_back(i);
      }
    }

    // Assume the initial times we are solving for are decent, and our equality
    // constraints are good.
    y = Eigen::VectorXd::Zero(derivatives.hessian.rows() +
                              active_constraints.size() + derivatives.A.rows());

    // Initialize our slack variable to the max constraint plus a little bit so
    // our inequality constraints are all satisfied at the start.
    for (int i = 0; i < derivatives.f.rows(); ++i) {
      y(derivatives.hessian.rows() - 1, 0) =
          std::max(y(derivatives.hessian.rows() - 1, 0), derivatives.f(i)) + 1;
    }

    // Initialize our inequality constraint legrange multipliers to be about the
    // size of the violation of the constraint.  The slope is ~1 for each, so it
    // should work out to be about the right order of magnitude.
    for (size_t i = 0; i < active_constraints.size(); ++i) {
      y(derivatives.gradient.rows() + i) =
          std::max(derivatives.f(active_constraints[i]), 1.0);
    }

    SOLVE_VLOG(my_solve_number_, 1)
        << "S initial is " << y(derivatives.hessian.rows() - 1, 0);
    SOLVE_VLOG(my_solve_number_, 1)
        << "Y initial is " << y.transpose().format(kHeavyFormat);
  }

  Problem::Derivatives derivatives = AddConstraintSlackVariable(
      problem->ComputeDerivatives(my_solve_number_, y, false, false,
                                  active_constraints),
      y);
  while (true) {
    SOLVE_VLOG(my_solve_number_, 1) << "Starting iteration " << iteration;
    // Now that we have all the derivatives computed, we can start to execute
    // this step.
    //
    // Note: we don't want to recompute them here.  For the first iteration, we
    // compute them outside the loop.  For any subsequent iterations, as part of
    // line search, we compute the derivatives to compute Rt.  So, we can just
    // reuse the final line search step's derivatives.

    solution_node = derivatives.solution_node;
    SOLVE_VLOG(my_solve_number_, 1) << "   y(" << iteration << ") is "
                                    << y.transpose().format(kHeavyFormat);

    // Round lambda so when we get super close to 0, we treat it just as 0. This
    // helps the matrix inversion be more accurate.  We also don't care about
    // accuracy that badly to need to care about lambda < 1e-12.  Our
    // constraints have a jacobian with a slope of ~1, so this would correspond
    // to a constraint violation of 1e-12 which is a rounding error at this
    // point given we only need < 0.5 ns precision.
    {
      Eigen::Ref<Eigen::VectorXd> lambda =
          y.block(derivatives.hessian.rows(), 0, derivatives.f.rows(), 1);
      for (int i = 0; i < lambda.rows(); ++i) {
        if (std::abs(lambda(i)) < 1e-12) {
          lambda(i) = 0.0;
        }
      }
    }

    PrintDerivatives(derivatives, y, "", 1);

    // Figure out our descent direction.
    Eigen::VectorXd dy;
    Eigen::VectorXd rt_orig;
    double t_inverse;
    std::tie(dy, t_inverse, rt_orig) =
        ConstrainedNewton(y, derivatives, iteration);
    SOLVE_VLOG(my_solve_number_, 1) << "   t " << 1.0 / t_inverse;

    double s = 1.0;

    const Eigen::Ref<const Eigen::VectorXd> x =
        y.block(0, 0, derivatives.hessian.rows(), 1);
    const Eigen::Ref<const Eigen::VectorXd> lambda =
        y.block(x.rows(), 0, derivatives.f.rows(), 1);
    Eigen::Ref<Eigen::VectorXd> dlambda =
        dy.block(x.rows(), 0, lambda.rows(), 1);

    const Eigen::Ref<const Eigen::VectorXd> v =
        y.block(x.rows() + lambda.rows(), 0, derivatives.A.rows(), 1);

    // Now, time to do line search.
    //
    // Start by keeping lambda positive.  Make sure our step doesn't let lambda
    // cross 0.
    for (int i = 0; i < dlambda.rows(); ++i) {
      if (lambda(i) + s * dlambda(i) < 0.0) {
        // Ignore tiny steps in lambda.  They cause issues when we get really
        // close to having our constraints met but haven't converged the rest of
        // the problem and start to run into rounding issues in the matrix solve
        // portion.
        if (dlambda(i) < 0.0 && dlambda(i) > -1e-12) {
          SOLVE_VLOG(my_solve_number_, 1)
              << "  lambda(" << i << ") " << lambda(i) << " + " << s << " * "
              << dlambda(i) << " -> s would be now " << -lambda(i) / dlambda(i);
          dlambda(i) = 0.0;
          SOLVE_VLOG(my_solve_number_, 1)
              << "  dy -> " << std::setprecision(12) << std::fixed
              << std::setfill(' ') << dy.transpose().format(kHeavyFormat);
          continue;
        }
        SOLVE_VLOG(my_solve_number_, 1)
            << "  lambda(" << i << ") " << lambda(i) << " + " << s << " * "
            << dlambda(i) << " -> s now " << -lambda(i) / dlambda(i);
        s = -lambda(i) / dlambda(i);
      }
    }

    s *= 0.999;

    SOLVE_VLOG(my_solve_number_, 1) << "  After lambda line search, s is " << s;

    const Eigen::Ref<const Eigen::VectorXd> dx = dy.block(0, 0, x.rows(), 1);
    const Eigen::Ref<const Eigen::VectorXd> dv =
        dy.block(x.rows() + lambda.rows(), 0, v.rows(), 1);

    SOLVE_VLOG(my_solve_number_, 3)
        << "  Initial step " << iteration << " -> " << std::setprecision(12)
        << std::fixed << std::setfill(' ')
        << dy.transpose().format(kHeavyFormat);
    SOLVE_VLOG(my_solve_number_, 3)
        << "   rt ->                                        "
        << std::setprecision(12) << std::fixed << std::setfill(' ')
        << rt_orig.transpose().format(kHeavyFormat);

    const double rt_orig_squared_norm = rt_orig.squaredNorm();

    // Now, do the rest of line search to make sure we don't increase Rt.
    Eigen::VectorXd rt;
    Eigen::VectorXd next_y;
    Problem::Derivatives next_derivatives;
    while (true) {
      next_y = y + s * dy;

      next_derivatives = AddConstraintSlackVariable(
          problem->ComputeDerivatives(my_solve_number_, next_y, true, false,
                                      active_constraints),
          next_y);

      rt = Rt(next_derivatives, next_y, t_inverse);

      const Eigen::Ref<const Eigen::VectorXd> next_x =
          next_y.block(0, 0, next_derivatives.hessian.rows(), 1);
      const Eigen::Ref<const Eigen::VectorXd> next_lambda =
          next_y.block(next_x.rows(), 0, next_derivatives.f.rows(), 1);

      const Eigen::Ref<const Eigen::VectorXd> next_v = next_y.block(
          next_x.rows() + next_lambda.rows(), 0, next_derivatives.A.rows(), 1);

      SOLVE_VLOG(my_solve_number_, 1)
          << "    next_rt(" << iteration << ") is " << rt.norm() << " -> "
          << std::setprecision(12) << std::fixed << std::setfill(' ')
          << rt.transpose().format(kHeavyFormat);

      PrintDerivatives(next_derivatives, next_y, "next_", 3);

      // TODO(austin): If we end up growing the residual too many times in a
      // row, then stop searching?  Quan thinks we can be more clever here.
      //
      // Keep stepping until we don't grow the residual, and until we don't
      // cross our constraint.  Since Axmb isn't perfectly convex, let ourselves
      // get worse until it gets close to converging.
      if (next_derivatives.f.maxCoeff() > 0.0) {
        SOLVE_VLOG(my_solve_number_, 1)
            << "   f_next > 0.0  -> " << next_derivatives.f.maxCoeff()
            << ", continuing line search.";
        s *= kBeta;
      } else if (next_derivatives.Axmb.squaredNorm() < 0.1 &&
                 rt.squaredNorm() >
                     std::pow(1.0 - kAlpha * s, 2.0) * rt_orig_squared_norm) {
        SOLVE_VLOG(my_solve_number_, 1)
            << "   |Rt| > |Rt+1| " << rt.norm() << " >  " << rt_orig.norm()
            << ", drt -> " << std::setprecision(12) << std::fixed
            << std::setfill(' ')
            << (rt_orig - rt).transpose().format(kHeavyFormat);
        s *= kBeta;
      } else {
        break;
      }
    }

    SOLVE_VLOG(my_solve_number_, 1)
        << "  Terminated line search with s " << s << ", " << rt.norm()
        << "(|Rt+1|) < " << rt_orig.norm() << "(|Rt|)";
    y = next_y;

    const Eigen::Ref<const Eigen::VectorXd> next_lambda =
        y.block(x.rows(), 0, lambda.rows(), 1);

    // See if we hit our convergence criteria.
    const double r_primal_squared_norm =
        rt.block(x.rows() + lambda.rows(), 0, v.rows(), 1).squaredNorm();
    SOLVE_VLOG(my_solve_number_, 1)
        << "  rt_next(" << iteration << ") is " << rt.norm() << " -> "
        << std::setprecision(12) << std::fixed << std::setfill(' ')
        << rt.transpose().format(kHeavyFormat);
    if (r_primal_squared_norm < kEpsilonF * kEpsilonF) {
      const double r_dual_squared_norm =
          rt.block(0, 0, x.rows(), 1).squaredNorm();
      if (r_dual_squared_norm < kEpsilonF * kEpsilonF) {
        const double next_nu =
            -(next_derivatives.f.transpose() * next_lambda)(0, 0);
        if (next_nu < kEpsilon) {
          break;
        } else {
          SOLVE_VLOG(my_solve_number_, 1)
              << "  nu(" << iteration << ") -> " << next_nu << " < " << kEpsilon
              << ", not done yet";
        }

      } else {
        SOLVE_VLOG(my_solve_number_, 1)
            << "  r_dual(" << iteration << ") -> "
            << std::sqrt(r_dual_squared_norm) << " < " << kEpsilonF
            << ", not done yet";
      }
    } else {
      SOLVE_VLOG(my_solve_number_, 1) << "  r_primal(" << iteration << ") -> "
                                      << std::sqrt(r_primal_squared_norm)
                                      << " < " << kEpsilonF << ", not done yet";
    }
    SOLVE_VLOG(my_solve_number_, 1)
        << "  step(" << iteration << ") "
        << (s * dy).transpose().format(kHeavyFormat);
    SOLVE_VLOG(my_solve_number_, 1) << " y(" << iteration << ") is now "
                                    << y.transpose().format(kHeavyFormat);

    // Very import, use the last set of derivatives we picked for our new y for
    // the next iteration.
    derivatives = std::move(next_derivatives);
    ++iteration;

    // Notify the problem that y updated.  This gives it a hook to pull digits
    // out of y if needed.
    problem->Update(my_solve_number_, y);

    // And finally, don't let us iterate forever.  If it isn't converging,
    // report back.
    if (iteration > max_iterations) {
      break;
    }
  }

  if (iteration > max_iterations && FLAGS_crash_on_solve_failure) {
    LOG(ERROR) << "Failed to converge on solve " << my_solve_number_;
    return std::nullopt;
  }

  return std::make_tuple(std::move(y), solution_node, iteration,
                         std::move(active_constraints));
}

void TimestampProblem::MaybeUpdateNodeMapping() {
  if (node_mapping_valid_) {
    return;
  }
  size_t states_count = 0;
  for (size_t i = 0; i < node_mapping_.size(); ++i) {
    if (live(i)) {
      node_mapping_[i] = states_count;
      ++states_count;
    } else {
      node_mapping_[i] = std::numeric_limits<size_t>::max();
    }
  }
  states_ = states_count;
  inequality_constraints_ = LiveConstraintsCount();
  node_mapping_valid_ = true;

  // Reset the cached state each time we solve.
  for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
    for (struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
      filter.pointer = NoncausalTimestampFilter::Pointer();
    }
  }
}

void TimestampProblem::Prepare(size_t solve_number) {
  MaybeUpdateNodeMapping();

  CHECK_GT(states(), 0u) << ": No live nodes to solve for.";

  for (size_t i = 0; i < points_.size(); ++i) {
    if (points_[i] != logger::BootTimestamp::max_time()) {
      SOLVE_VLOG(solve_number, 2)
          << "Solving for node " << i << " at " << points_[i];
    }
  }
}

void TimestampProblem::Update(size_t solve_number,
                              Eigen::Ref<Eigen::VectorXd> data) {
  // We are doing all our math with both an int64 base and a double offset.
  // This lets us handle large offsets while retaining precision down to the
  // nanosecond easily.
  //
  // Some problems start out with a poor initial solution.  This is
  // especially true for the first solution.  Because we control the solver,
  // as we determine that the double is getting too big, we can move that
  // information to the int64 base clock.  Threshold this to not be *too*
  // big since it makes it hard to debug as the data keeps jumping around.
  for (size_t j = 0; j < size(); ++j) {
    const size_t solution_index = NodeToFullSolutionIndex(j);
    if (live(j)) {
      bool updated = false;
      int64_t dsolution = 0;
      if (std::abs(data(solution_index)) > 100) {
        dsolution = static_cast<int64_t>(std::round(data(solution_index)));
        base_clock_[j].time += chrono::nanoseconds(dsolution);
        data(solution_index) -= dsolution;
        updated = true;
      }

      SOLVE_VLOG(solve_number, 2)
          << "    live " << points_[j].time << " vs solution "
          << base_clock_[j].time +
                 std::chrono::nanoseconds(static_cast<int64_t>(
                     std::round(data(NodeToFullSolutionIndex(j)))))
          << " "
          << (data(NodeToFullSolutionIndex(j)) -
              std::round(data(NodeToFullSolutionIndex(j))))
          << " (unrounded: " << data(NodeToFullSolutionIndex(j)) << ")"
          << (updated ? (std::string(" updated ") + std::to_string(dsolution))
                      : std::string(""))
          << " dt "
          << (points_[j].time -
              (base_clock_[j].time +
               std::chrono::nanoseconds(static_cast<int64_t>(
                   std::round(data(NodeToFullSolutionIndex(j)))))))
                 .count()
          << "ns";
    } else {
      SOLVE_VLOG(solve_number, 2)
          << "    dead  " << aos::monotonic_clock::min_time;
      SOLVE_VLOG(solve_number, 2)
          << "    dead  " << aos::monotonic_clock::min_time;
    }
  }
}

void TimestampProblem::Debug() {
  MaybeUpdateNodeMapping();

  std::vector<std::vector<std::string>> gradients(
      clock_offset_filter_for_node_.size());
  for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
    for (const struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
      if (live(i) && live(filter.b_index)) {
        // TODO(austin): This should be right, but I haven't gone and spent a
        // bunch of time making sure it all matches perfectly.  We aren't
        // hitting this anymore.  I'm also likely the one who will be debugging
        // it next and would rather spend the time debugging it when I get a bug
        // report.
        if (FLAGS_bounds_offset_error) {
          gradients[i].emplace_back(
              std::string("- ") +
              filter.filter->DebugOffsetError(
                  nullptr, NoncausalTimestampFilter::Pointer(), base_clock_[i],
                  0.0, base_clock_[filter.b_index], 0.0, i, filter.b_index));
          gradients[filter.b_index].emplace_back(
              filter.filter->DebugOffsetError(
                  nullptr, NoncausalTimestampFilter::Pointer(), base_clock_[i],
                  0.0, base_clock_[filter.b_index], 0.0, i, filter.b_index));
        } else {
          gradients[i].emplace_back(
              std::string("- ") +
              filter.filter->DebugOffsetError(
                  filter.b_filter, NoncausalTimestampFilter::Pointer(),
                  base_clock_[i], 0.0, base_clock_[filter.b_index], 0.0, i,
                  filter.b_index));
          gradients[filter.b_index].emplace_back(
              filter.filter->DebugOffsetError(
                  filter.b_filter, NoncausalTimestampFilter::Pointer(),
                  base_clock_[i], 0.0, base_clock_[filter.b_index], 0.0, i,
                  filter.b_index));
        }
      }
    }
  }

  for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
    LOG(INFO) << (live(i) ? "live" : "dead") << " Grad[" << i << "] = "
              << (gradients[i].empty() ? std::string("0.0")
                                       : absl::StrJoin(gradients[i], " + "));
  }

  for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
    LOG(INFO) << (live(i) ? "live" : "dead") << " base_clock[" << i
              << "] = " << base_clock_[i];
  }
}

std::optional<std::optional<const std::tuple<distributed_clock::time_point,
                                             std::vector<BootTimestamp>> *>>
InterpolatedTimeConverter::QueueNextTimestamp() {
  std::optional<std::optional<
      std::tuple<distributed_clock::time_point, std::vector<BootTimestamp>>>>
      next_time = NextTimestamp();
  if (!next_time.has_value()) {
    VLOG(1) << "Error in processing timestamps.";
    return std::nullopt;
  }
  if (!next_time.value().has_value()) {
    VLOG(1) << "Last timestamp, calling it quits";
    std::optional<std::optional<const std::tuple<distributed_clock::time_point,
                                                 std::vector<BootTimestamp>> *>>
        result;
    result.emplace(std::nullopt);
    // Check that C++ actually works how we think it does...
    CHECK(result.has_value());
    at_end_ = true;
    return result;
  }

  VLOG(1) << "Fetched next timestamp while solving: "
          << std::get<0>(**next_time) << " ->";
  for (BootTimestamp t : std::get<1>(**next_time)) {
    VLOG(1) << "  " << t;
  }

  CHECK_EQ(node_count_, std::get<1>(**next_time).size());

  if (times_.empty()) {
    for (BootTimestamp t : std::get<1>(**next_time)) {
      CHECK_EQ(t.boot, 0u);
    }
  } else {
    bool rebooted = false;
    for (size_t i = 0; i < node_count_; ++i) {
      if (std::get<1>(times_.back())[i].boot !=
          std::get<1>(**next_time)[i].boot) {
        rebooted = true;
        break;
      }
    }
    if (rebooted) {
      CHECK(reboot_found_);
      if (VLOG_IS_ON(2)) {
        VLOG(2) << "Notified reboot of";
        size_t node_index = 0;
        for (logger::BootTimestamp t : std::get<1>(**next_time)) {
          VLOG(2) << "  Node " << node_index << " " << t;
          ++node_index;
        }
      }
      reboot_found_(std::get<0>(**next_time), std::get<1>(**next_time));
    }
  }
  times_.emplace_back(std::move(**next_time));
  return &times_.back();
}

void InterpolatedTimeConverter::ObserveTimePassed(
    distributed_clock::time_point time) {
  // Keep at least 500 points and time_estimation_buffer_seconds seconds of
  // time.  This should be enough to handle any reasonable amount of history.
  while (true) {
    if (times_.size() < kHistoryMinCount) {
      return;
    }
    if (std::get<0>(times_[1]) + time_estimation_buffer_seconds_ > time) {
      VLOG(1) << "Not popping because "
              << std::get<0>(times_[1]) + time_estimation_buffer_seconds_
              << " > " << time;
      return;
    }

    VLOG(1) << "Popping sample because " << times_.size() << " > "
            << kHistoryMinCount << " && " << std::get<0>(times_[1]) << " < "
            << time - time_estimation_buffer_seconds_;
    times_.pop_front();
    have_popped_ = true;
  }
}

distributed_clock::time_point ToDistributedClock(
    distributed_clock::time_point d0, distributed_clock::time_point d1,
    monotonic_clock::time_point t0, monotonic_clock::time_point t1,
    monotonic_clock::time_point time) {
  const chrono::nanoseconds dt = (t1 - t0);

  CHECK_NE(dt.count(), 0u) << " t0 " << t0 << " t1 " << t1 << " d0 " << d0
                           << " d1 " << d1 << " looking up monotonic " << time;
  // Basic interpolation between 2 points look like
  //  p0.d + (t - p0.t) * (p1.d - p0.d) / (p1.t - p0.t)
  // This can be multiplied out with integer arithmetic to get exact results.
  // Since we are using integer arithmetic, we want to round to the nearest, not
  // towards 0.  To do that, we want to add half of the denominator when > 0,
  // and subtract when < 0 so we round correctly.  Multiply before dividing so
  // we don't round early, and use 128 bit arithmetic to guarantee that 64 bit
  // multiplication fits.
  absl::int128 numerator =
      absl::int128((time - t0).count()) * absl::int128((d1 - d0).count());
  numerator += numerator > 0 ? absl::int128(dt.count() / 2)
                             : -absl::int128(dt.count() / 2);
  return d0 + std::chrono::nanoseconds(
                  static_cast<int64_t>(numerator / absl::int128(dt.count())));
}

distributed_clock::time_point InterpolatedTimeConverter::ToDistributedClock(
    size_t node_index, BootTimestamp time) {
  CHECK_LT(node_index, node_count_);
  // If there is only one node, time estimation makes no sense.  Just return
  // unity time.
  if (node_count_ == 1u) {
    return distributed_clock::epoch() + time.time_since_epoch();
  }

  // Make sure there are enough timestamps in the queue.
  QueueUntil(
      [time, node_index](const std::tuple<distributed_clock::time_point,
                                          std::vector<BootTimestamp>> &t) {
        return std::get<1>(t)[node_index] < time;
      });

  // Before the beginning needs to have 0 slope otherwise time jumps when
  // timestamp 2 happens.
  if (times_.size() == 1u || time < std::get<1>(times_[0])[node_index]) {
    if (time < std::get<1>(times_[0])[node_index]) {
      CHECK(!have_popped_)
          << ": Trying to interpolate time " << time
          << " but we have forgotten the relevant points already.";
    }
    CHECK_EQ(time.boot, std::get<1>(times_[0])[node_index].boot);
    const distributed_clock::time_point result =
        time.time - std::get<1>(times_[0])[node_index].time +
        std::get<0>(times_[0]);
    VLOG(3) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
            << result;
    return result;
  }

  // Now, find the corresponding timestamps.  Search from the back since that's
  // where most of the times we care about will be.
  auto search = std::upper_bound(
      times_.rbegin() + 1, times_.rend() - 1, time,
      [node_index](BootTimestamp time,
                   const std::tuple<distributed_clock::time_point,
                                    std::vector<logger::BootTimestamp>> &t) {
        return std::get<1>(t)[node_index] <= time;
      });

  // Interpolate with the two of these.
  const std::tuple<distributed_clock::time_point,
                   std::vector<logger::BootTimestamp>> &p0 = *search;
  const std::tuple<distributed_clock::time_point,
                   std::vector<logger::BootTimestamp>> &p1 = *(search - 1);

  const distributed_clock::time_point d0 = std::get<0>(p0);
  const distributed_clock::time_point d1 = std::get<0>(p1);

  const BootTimestamp t0 = std::get<1>(p0)[node_index];
  const BootTimestamp t1 = std::get<1>(p1)[node_index];

  if (time > t1) {
    const distributed_clock::time_point result = (time.time - t1.time) + d1;
    VLOG(3) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
            << result;
    return result;
  }

  if (t0.boot != t1.boot) {
    if (t0.boot == time.boot) {
      const distributed_clock::time_point result = (time.time - t0.time) + d0;
      VLOG(3) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
              << result;
      return result;
    } else if (t1.boot == time.boot) {
      const distributed_clock::time_point result = (time.time - t1.time) + d1;
      VLOG(3) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
              << result;
      return result;
    } else {
      LOG(FATAL) << t0 << " <= " << time << " <= " << t1;
    }
  }

  const distributed_clock::time_point result =
      message_bridge::ToDistributedClock(d0, d1, t0.time, t1.time, time.time);

  VLOG(3) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
          << result;
  return result;
}

BootTimestamp InterpolatedTimeConverter::FromDistributedClock(
    size_t node_index, distributed_clock::time_point time, size_t boot_count) {
  CHECK_LT(node_index, node_count_);
  // If there is only one node, time estimation makes no sense.  Just return
  // unity time.
  if (node_count_ == 1u) {
    return BootTimestamp::epoch() + time.time_since_epoch();
  }

  // Make sure there are enough timestamps in the queue.
  QueueUntil([time](const std::tuple<distributed_clock::time_point,
                                     std::vector<BootTimestamp>> &t) {
    return std::get<0>(t) < time;
  });

  if (times_.size() == 1u || time < std::get<0>(times_[0])) {
    if (time < std::get<0>(times_[0])) {
      CHECK(!have_popped_)
          << ": Trying to interpolate time " << time
          << " but we have forgotten the relevant points already.";
    }
    monotonic_clock::time_point result =
        time - std::get<0>(times_[0]) + std::get<1>(times_[0])[node_index].time;
    VLOG(3) << "FromDistributedClock(" << node_index << ", " << time << ", "
            << boot_count << ") -> " << result;
    return {.boot = std::get<1>(times_[0])[node_index].boot, .time = result};
  }

  // Now, find the corresponding timestamps.  Search from the back since that's
  // where most of the times we care about will be.
  auto search = std::upper_bound(
      times_.rbegin() + 1, times_.rend() - 1, time,
      [](distributed_clock::time_point time,
         const std::tuple<distributed_clock::time_point,
                          std::vector<logger::BootTimestamp>> &t) {
        // If we are searching across a reboot, we want both
        // the before and after time.  We will be asked to
        // solve for the after, so make sure when a time
        // matches exactly, we pick the time before, not the
        // time after.
        return std::get<0>(t) < time;
      });

  const std::tuple<distributed_clock::time_point,
                   std::vector<logger::BootTimestamp>> &p0 = *search;
  const std::tuple<distributed_clock::time_point,
                   std::vector<logger::BootTimestamp>> &p1 = *(search - 1);

  // Interpolate with the two of these.
  const distributed_clock::time_point d0 = std::get<0>(p0);
  const distributed_clock::time_point d1 = std::get<0>(p1);
  const BootTimestamp t0 = std::get<1>(p0)[node_index];
  const BootTimestamp t1 = std::get<1>(p1)[node_index];

  if (time == d1) {
    if (boot_count == t1.boot) {
      const BootTimestamp result = t1 + (time - d1);
      VLOG(3) << "FromDistributedClock(" << node_index << ", " << time << ", "
              << boot_count << ") -> " << result;
      return result;
    } else {
      CHECK_EQ(boot_count, t0.boot);
      const BootTimestamp result = t0 + (time - d0);
      VLOG(3) << "FromDistributedClock(" << node_index << ", " << time << ", "
              << boot_count << ") -> " << result;
      return result;
    }
  }

  if (time > d1) {
    const BootTimestamp result = t1 + (time - d1);
    VLOG(3) << "FromDistributedClock(" << node_index << ", " << time << ", "
            << boot_count << ") -> " << result;
    return result;
  }

  if (t0.boot != t1.boot) {
    const BootTimestamp result = t0 + (time - d0);
    VLOG(3) << "FromDistributedClock(" << node_index << ", " << time << ", "
            << boot_count << ") -> " << result;
    return result;
  }

  const chrono::nanoseconds dd = d1 - d0;

  CHECK_NE(dd.count(), 0u) << " t0 " << t0 << " t1 " << t1 << "d0 " << d0
                           << " d1 " << d1 << " looking up distributed "
                           << time;

  // Basic interpolation between 2 points look like
  //  p0.t + (t - p0.d) * (p1.t - p0.t) / (p1.d - p0.d)
  // This can be multiplied out with integer arithmetic to get exact results.
  // Since we are using integer arithmetic, we want to round to the nearest, not
  // towards 0.  To do that, we want to add half of the denominator when > 0,
  // and subtract when < 0 so we round correctly.  Multiply before dividing so
  // we don't round early, and use 128 bit arithmetic to guarantee that 64 bit
  // multiplication fits.
  absl::int128 numerator = absl::int128((time - d0).count()) *
                           absl::int128((t1.time - t0.time).count());
  numerator += numerator > 0 ? absl::int128(dd.count() / 2)
                             : -absl::int128(dd.count() / 2);

  const monotonic_clock::time_point result =
      t0.time + std::chrono::nanoseconds(
                    static_cast<int64_t>(numerator / absl::int128(dd.count())));
  VLOG(3) << "FromDistributedClock(" << node_index << ", " << time << ", "
          << boot_count << ") -> " << result;
  return {.boot = t0.boot, .time = result};
}

MultiNodeNoncausalOffsetEstimator::MultiNodeNoncausalOffsetEstimator(
    const Configuration *configuration,
    const Configuration *logged_configuration,
    std::shared_ptr<const logger::Boots> boots, bool skip_order_validation,
    chrono::nanoseconds time_estimation_buffer_seconds)
    : InterpolatedTimeConverter(!configuration::MultiNode(logged_configuration)
                                    ? 1u
                                    : logged_configuration->nodes()->size(),
                                time_estimation_buffer_seconds),
      configuration_(configuration),
      logged_configuration_(logged_configuration),
      boots_(boots),
      skip_order_validation_(skip_order_validation) {
  const bool multi_node = configuration::MultiNode(logged_configuration);
  if (!boots_ && !multi_node) {
    // This is a super old log.  Fake out boots by making them up.
    LOG(WARNING) << "Old single node log without boot UUIDs, generating a "
                    "random boot UUID.";
    std::shared_ptr<logger::Boots> boots = std::make_shared<logger::Boots>();
    const UUID random_boot_uuid = UUID::Random();
    boots->boot_count_map.emplace(random_boot_uuid.ToString(), 0);
    boots->boots =
        std::vector<std::vector<std::string>>{{random_boot_uuid.ToString()}};
    boots_ = boots;
  }

  CHECK(boots_) << ": Missing boots for " << NodesCount();
  CHECK_EQ(boots_->boots.size(), NodesCount());
  filters_per_node_.resize(NodesCount());
  last_monotonics_.resize(NodesCount(), BootTimestamp::epoch());
  if (FLAGS_timestamps_to_csv && multi_node) {
    fp_ = fopen(CsvPath("timestamp_noncausal_offsets.csv").c_str(), "w");
    fprintf(fp_, "# distributed");
    for (const Node *node : configuration::GetNodes(logged_configuration)) {
      fprintf(fp_, ", %s", node->name()->c_str());
    }
    fprintf(fp_, "\n");
    filter_fps_.resize(NodesCount());
    for (auto &filter_fp : filter_fps_) {
      filter_fp.resize(NodesCount(), nullptr);
    }
    sample_fps_.resize(NodesCount());
    for (auto &sample_fp : sample_fps_) {
      sample_fp.resize(NodesCount(), nullptr);
    }

    node_samples_.resize(NodesCount());
    for (NodeSamples &node_samples : node_samples_) {
      node_samples.nodes.resize(NodesCount());
    }

    source_node_index_ = configuration::SourceNodeIndex(logged_configuration);
  }
}

bool MultiNodeNoncausalOffsetEstimator::FlushAndClose(bool destructor) {
  // Write out all the data in our filters.
  FlushAllSamples(true);
  if (fp_) {
    fclose(fp_);
    fp_ = NULL;
  }

  if (filter_fps_.size() != 0 && !destructor) {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      for (const auto &filter : filters) {
        while (true) {
          std::optional<std::tuple<logger::BootTimestamp, logger::BootDuration>>
              sample = filter.filter->Consume();
          if (!sample) {
            break;
          }
          WriteFilter(filter.filter, *sample);
        }
      }
      ++node_a_index;
    }
  }

  if (filter_fps_.size() != 0) {
    for (std::vector<FILE *> &filter_fp : filter_fps_) {
      for (FILE *&fp : filter_fp) {
        if (fp != nullptr) {
          fclose(fp);
        }
      }
    }
    filter_fps_.clear();
  }
  if (sample_fps_.size() != 0) {
    for (std::vector<FILE *> &filter_fp : sample_fps_) {
      for (FILE *&fp : filter_fp) {
        if (fp != nullptr) {
          fclose(fp);
        }
      }
    }
    sample_fps_.clear();
  }

  if (all_done_) {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      for (const auto &filter : filters) {
        std::optional<std::tuple<BootTimestamp, BootDuration>> next =
            filter.filter->Consume();
        if (next) {
          LOG(ERROR) << "MultiNodeNoncausalOffsetEstimator reported all "
                        "done, but "
                     << node_a_index << " -> " << filter.b_index
                     << " found more data at time " << std::get<0>(*next)
                     << ".  Time estimation was silently wrong.";
          if (!skip_order_validation_) {
            return false;
          }
        }
      }
      ++node_a_index;
    }
  }

  // Make sure everything is flushed to disk.
  if (!node_samples_.empty()) {
    for (NodeSamples &node : node_samples_) {
      for (SingleNodeSamples &timestamps : node.nodes) {
        if (!timestamps.messages.empty()) {
          LOG(ERROR) << "Timestamps still remaining.";
          return false;
        }
      }
    }
  }
  return true;
}

MultiNodeNoncausalOffsetEstimator::~MultiNodeNoncausalOffsetEstimator() {
  const bool success = FlushAndClose(true);
  if (!non_fatal_destructor_checks_) {
    CHECK(success);
  }
}

UUID MultiNodeNoncausalOffsetEstimator::boot_uuid(size_t node_index,
                                                  size_t boot_count) {
  CHECK(boots_);
  CHECK_LT(node_index, boots_->boots.size());
  if (boot_count < boots_->boots[node_index].size()) {
    return UUID::FromString(boots_->boots[node_index][boot_count]);
  } else {
    return UUID::Random();
  }
}

void MultiNodeNoncausalOffsetEstimator::Start(
    SimulatedEventLoopFactory *factory) {
  if (!configuration::MultiNode(factory->configuration())) {
    return;
  }
  std::vector<monotonic_clock::time_point> times;
  for (const Node *node : configuration::GetNodes(factory->configuration())) {
    times.emplace_back(factory->GetNodeEventLoopFactory(node)->monotonic_now());
  }
  Start(times);
}

void MultiNodeNoncausalOffsetEstimator::Start(
    std::vector<monotonic_clock::time_point> times) {
  if (FLAGS_timestamps_to_csv) {
    std::fstream s(CsvPath("timestamp_noncausal_starttime.csv").c_str(),
                   s.trunc | s.out);
    CHECK(s.is_open());
    for (const Node *node : configuration::GetNodes(configuration())) {
      const size_t node_index =
          configuration::GetNodeIndex(configuration(), node);
      s << node->name()->string_view() << ", " << std::setprecision(12)
        << std::fixed
        << chrono::duration<double>(times[node_index].time_since_epoch())
               .count()
        << "\n";
    }
  }
}

message_bridge::NoncausalOffsetEstimator *
MultiNodeNoncausalOffsetEstimator::GetFilter(const Node *node_a,
                                             const Node *node_b) {
  CHECK_NE(node_a, node_b);
  CHECK_EQ(configuration::GetNode(configuration(), node_a), node_a);
  CHECK_EQ(configuration::GetNode(configuration(), node_b), node_b);

  if (node_a > node_b) {
    return GetFilter(node_b, node_a);
  }

  auto tuple = std::make_tuple(node_a, node_b);

  auto it = filters_.find(tuple);

  if (it == filters_.end()) {
    auto &x = filters_
                  .emplace(tuple, message_bridge::NoncausalOffsetEstimator(
                                      node_a, node_b))
                  .first->second;

    const size_t node_a_index =
        configuration::GetNodeIndex(logged_configuration_, node_a);
    const size_t node_b_index =
        configuration::GetNodeIndex(logged_configuration_, node_b);

    // TODO(austin): Do a better job documenting which node is which here.
    filters_per_node_[node_a_index].emplace_back(
        x.GetFilter(node_a), node_b_index, x.GetFilter(node_b));
    filters_per_node_[node_b_index].emplace_back(
        x.GetFilter(node_b), node_a_index, x.GetFilter(node_a));
    return &x;
  } else {
    return &it->second;
  }
}

void MultiNodeNoncausalOffsetEstimator::SetTimestampMappers(
    std::vector<logger::TimestampMapper *> timestamp_mappers) {
  CHECK_EQ(timestamp_mappers.size(), NodesCount());
  filters_per_channel_.resize(timestamp_mappers.size());

  // Pre-build all the filters.  Why not?
  for (const Node *node : configuration::GetNodes(logged_configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    filters_per_channel_[node_index].resize(
        logged_configuration()->channels()->size(), nullptr);
    for (size_t channel_index = 0;
         channel_index < logged_configuration()->channels()->size();
         ++channel_index) {
      const Channel *channel =
          logged_configuration()->channels()->Get(channel_index);

      if (!configuration::ChannelIsSendableOnNode(channel, node) &&
          configuration::ChannelIsReadableOnNode(channel, node)) {
        // We've got a message which is being forwarded to this node.
        const Node *source_node = configuration::GetNode(
            configuration(), channel->source_node()->string_view());
        filters_per_channel_[node_index][channel_index] =
            GetFilter(configuration()->nodes()->Get(node_index), source_node);
      }
    }
  }

  size_t node_index = 0;
  for (logger::TimestampMapper *timestamp_mapper : timestamp_mappers) {
    if (timestamp_mapper != nullptr) {
      CHECK(!timestamp_mapper->started())
          << ": Timestamps queued before we registered the timestamp hooks.";
      timestamp_mapper->set_timestamp_callback(
          [this, node_index](logger::TimestampedMessage *msg) {
            if (msg->monotonic_remote_time != BootTimestamp::min_time()) {
              // Got a forwarding timestamp!
              NoncausalOffsetEstimator *filter =
                  filters_per_channel_[node_index][msg->channel_index];
              CHECK_NOTNULL(filter);
              const Node *node = configuration()->nodes()->Get(node_index);

              // Call the correct method depending on if we are the forward or
              // reverse direction here.
              filter->Sample(node, msg->monotonic_event_time,
                             msg->monotonic_remote_time);

              if (!node_samples_.empty()) {
                const size_t sending_node_index =
                    source_node_index_[msg->channel_index];
                // The message went from node sending_node_index to
                // node_index.  monotonic_remote_time is the time it was sent,
                // and monotonic_event_time was the time it was received.
                node_samples_[node_index]
                    .nodes[sending_node_index]
                    .messages.emplace(std::make_pair(
                        msg->monotonic_event_time, msg->monotonic_remote_time));
              }

              if (msg->monotonic_timestamp_time != BootTimestamp::min_time()) {
                // TODO(austin): This assumes that this timestamp is only logged
                // on the node which sent the data.  That is correct for now,
                // but should be explicitly checked somewhere.
                filter->ReverseSample(node, msg->monotonic_event_time,
                                      msg->monotonic_timestamp_time);

                if (!node_samples_.empty()) {
                  const size_t sending_node_index =
                      source_node_index_[msg->channel_index];
                  // The timestamp then went back from node node_index to
                  // sending_node_index.  monotonic_event_time is the time it
                  // was sent, and monotonic_timestamp_time was the time it was
                  // received.
                  node_samples_[sending_node_index]
                      .nodes[node_index]
                      .messages.emplace(
                          std::make_pair(msg->monotonic_timestamp_time,
                                         msg->monotonic_event_time));
                }
              }
            }
          });
    }
    ++node_index;
  }

  timestamp_mappers_ = std::move(timestamp_mappers);
}

TimeComparison CompareTimes(const std::vector<BootTimestamp> &ta,
                            const std::vector<BootTimestamp> &tb) {
  if (ta.size() != tb.size() || ta.empty()) {
    return TimeComparison::kInvalid;
  }
  bool is_less = false;
  bool is_greater = false;
  bool is_eq = true;
  bool some_eq = false;
  for (size_t i = 0; i < ta.size(); ++i) {
    if (tb[i] == BootTimestamp::min_time() ||
        ta[i] == BootTimestamp::min_time()) {
      continue;
    }
    if (ta[i].boot != tb[i].boot) {
      continue;
    }
    if (ta[i].time < tb[i].time) {
      is_less = true;
      is_eq = false;
    } else if (ta[i].time > tb[i].time) {
      is_greater = true;
      is_eq = false;
    } else {
      some_eq = true;
    }
  }

  if (is_eq) {
    return TimeComparison::kEq;
  }
  if (some_eq) {
    // if at least one, but not all, are equal, then consider this invalid
    // This represents a case where the solution on at least one node
    // has not moved forward
    return TimeComparison::kInvalid;
  }
  if (is_less && !is_greater) {
    return TimeComparison::kBefore;
  }
  if (!is_less && is_greater) {
    return TimeComparison::kAfter;
  } else {
    // If we have elements which are both < and >, that is a problem. We are
    // trying to order timestamps with this code, and equality doesn't provide
    // an ordering.
    return TimeComparison::kInvalid;
  }
}

chrono::nanoseconds MaxElapsedTime(const std::vector<BootTimestamp> &ta,
                                   const std::vector<BootTimestamp> &tb) {
  CHECK_EQ(ta.size(), tb.size());
  CHECK(!ta.empty());
  bool first = true;
  chrono::nanoseconds dt;
  for (size_t i = 0; i < ta.size(); ++i) {
    // Skip any invalid timestamps.
    if (ta[i] == BootTimestamp::min_time() ||
        tb[i] == BootTimestamp::min_time()) {
      continue;
    }

    if (ta[i].boot == tb[i].boot) {
      const chrono::nanoseconds dti = tb[i].time - ta[i].time;
      if (first || dti > dt) {
        dt = dti;
      }
      first = false;
    }
  }
  CHECK(!first);
  return dt;
}

chrono::nanoseconds InvalidDistance(const std::vector<BootTimestamp> &ta,
                                    const std::vector<BootTimestamp> &tb) {
  // Use an int128 so we have no concern about number of times or size of the
  // difference.
  absl::int128 sum = 0;
  for (size_t i = 0; i < ta.size(); ++i) {
    if (ta[i] == BootTimestamp::min_time() ||
        tb[i] == BootTimestamp::min_time()) {
      continue;
    }
    if (ta[i].boot == tb[i].boot) {
      sum += (ta[i].time - tb[i].time).count();
    }
  }
  // Pick the direction and sign to return.
  if (sum < 0) {
    return MaxElapsedTime(tb, ta);
  } else {
    return MaxElapsedTime(ta, tb);
  }
}

// Class to efficiently track up to 64 bit sets.  It uses a uint64 as the
// backing store, and ffs() to find the first bit set efficiently so we can
// iterate through the set.
class BitSet64 {
 public:
  BitSet64(size_t size) : size_(size) {
    // Cheat a bit...  Some of the math below assumes that 1 << size fits in a
    // 64 bit number, and it isn't worth fixing this assumption.
    CHECK_LE(size, (sizeof(uint64_t) * 8) - 1);
  }

  size_t size() const { return size_; }

  void Set(size_t index, bool val) {
    if (val) {
      data_ |= (1 << index);
    } else {
      data_ &= ~(1 << index);
    }
  }
  bool Get(size_t index) const { return (data_ & (1 << index)) != 0; }

  bool operator==(const BitSet64 &other) const {
    return size_ == other.size_ && data_ == other.data_;
  }

  bool operator!=(const BitSet64 &other) const {
    return size_ != other.size_ || data_ != other.data_;
  }

  // Flips all the indices in the set.
  BitSet64 operator~() const {
    // Only flip the bits in the set to keep == working above.
    BitSet64 result(*this);
    result.data_ = data_ ^ ((1u << size_) - 1u);
    return result;
  }

  // Returns a set for which only the bits which are set in both inputs sets are
  // set.
  BitSet64 operator&(const BitSet64 &other) const {
    BitSet64 result(*this);
    result.data_ = data_ & other.data_;
    return result;
  }

  // Returns the first bit set at or after start.  Returns size() if no more
  // bits are set.
  size_t FirstBitSet(size_t start) const {
    static_assert(sizeof(uintmax_t) == sizeof(int64_t),
                  "ffsimax is the wrong size");
    const int ffs = __builtin_ffsll(data_ & (~((1u << start) - 1u)));
    if (ffs == 0) {
      return size_;
    }

    return ffs - 1;
  }

 private:
  size_t size_;
  uint64_t data_ = 0;
};

void MultiNodeNoncausalOffsetEstimator::CheckGraph() {
  // Question doesn't make sense.
  if (NodesCount() == 1) {
    return;
  }

  // Find a starting point.
  BitSet64 all_nodes(NodesCount());
  size_t node_a_index = 0;
  bool found_start = false;
  for (const auto &filters : filters_per_node_) {
    for (const auto &filter : filters) {
      // Pick an initial seed.
      all_nodes.Set(filter.b_index, true);
      all_nodes.Set(node_a_index, true);
      found_start = true;
      break;
    }
    ++node_a_index;
  }

  CHECK(found_start) << ": Failed to find any connected nodes in a graph of "
                     << NodesCount();

  // The set of nodes we have visited.
  BitSet64 visited_set(all_nodes.size());
  while (true) {
    // Compute the set of all nodes which are new and haven't been visited to
    // track down dependencies.
    const BitSet64 new_nodes = all_nodes & ~visited_set;

    // And then, before changing it, record the list of nodes we've traversed as
    // now already visited.
    visited_set = all_nodes;

    for (size_t i = new_nodes.FirstBitSet(0); i < new_nodes.size();
         i = new_nodes.FirstBitSet(i + 1)) {
      for (const auto &filter : filters_per_node_[i]) {
        all_nodes.Set(i, true);
        all_nodes.Set(filter.b_index, true);
      }
    }

    if (visited_set == all_nodes) {
      // No change, abort.
      break;
    }
  }

  // Now, see if we found them all.
  const BitSet64 full_set = ~BitSet64(all_nodes.size());
  if (all_nodes != full_set) {
    // Nope, print them out and explode.
    const BitSet64 orphaned_nodes = (~all_nodes) & full_set;
    for (size_t i = orphaned_nodes.FirstBitSet(0); i < orphaned_nodes.size();
         i = orphaned_nodes.FirstBitSet(i + 1)) {
      LOG(ERROR) << "Node " << i << " is orphaned";
    }

    LOG(FATAL) << "Found orphaned nodes";
  }
}

TimestampProblem MultiNodeNoncausalOffsetEstimator::MakeProblem() {
  // Build up the problem for all valid timestamps.
  TimestampProblem problem(NodesCount());

  bool found_start = false;
  BitSet64 traversed_nodes(problem.size());
  BitSet64 all_live_nodes(problem.size());
  const BitSet64 all_nodes = ~BitSet64(problem.size());

  if (time_estimation_buffer_seconds_ != chrono::seconds(0)) {
    for (size_t node_index = 0; node_index < timestamp_mappers_.size();
         ++node_index) {
      if (timestamp_mappers_[node_index] != nullptr) {
        // Make sure we have enough data queued such that if we are going to
        // have a timestamp on this filter, we do have a timestamp queued.
        timestamp_mappers_[node_index]->QueueFor(
            time_estimation_buffer_seconds_);
      }
    }
  }

  {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      for (const auto &filter : filters) {
        if (filter.filter->timestamps_size() > 0u) {
          if (!found_start) {
            // Pick an initial seed.
            traversed_nodes.Set(node_a_index, true);
            traversed_nodes.Set(filter.b_index, true);
            found_start = true;
          }
          all_live_nodes.Set(node_a_index, true);
          all_live_nodes.Set(filter.b_index, true);
          problem.add_clock_offset_filter(node_a_index, filter.filter,
                                          filter.b_index, filter.b_filter);

          if (time_estimation_buffer_seconds_ == chrono::seconds(0)) {
            continue;
          }

          if (timestamp_mappers_[node_a_index] != nullptr) {
            // Now, we have cases at startup where we have a couple of points
            // followed by a long gap, followed by the body of the data.  We are
            // extrapolating, then adding the new data, and finding that time
            // was frozen already.
            //
            // The fix is to make sure we queue a couple of seconds past the
            // second point in the line segment, and we always make sure to load
            // the line segment.
            //
            // But, there are filters which have no data in them.  We don't want
            // to queue those until we hit 2, because that'll force us to read
            // everything into memory.  So, only queue for the filters which
            // have data in them.

            size_t node_b_index = configuration::GetNodeIndex(
                timestamp_mappers_[node_a_index]->configuration(),
                configuration::GetNode(
                    timestamp_mappers_[node_a_index]->configuration(),
                    filter.filter->node_b()));

            // Timestamps can come from either node.  When a message is
            // delivered to a node, it can have the timestamp time attached to
            // that message as well.  That means that we have to queue both
            // nodes until we have 2 unobserved points from both nodes.
            timestamp_mappers_[node_a_index]->QueueUntilCondition(
                [&filter]() { return filter.filter->has_unobserved_line(); });

            // Sometimes, we literally have no logs for the reverse direction.
            // So don't sweat it.
            if (timestamp_mappers_[node_b_index] != nullptr) {
              timestamp_mappers_[node_b_index]->QueueUntilCondition(
                  [&filter]() { return filter.filter->has_unobserved_line(); });
            }

            // If we actually found a line, make sure to buffer to the desired
            // distance past that last point so the filter doesn't try to
            // invalidate the point.  Do this for both nodes to pick up all the
            // timestamps.
            if (filter.filter->has_unobserved_line()) {
              timestamp_mappers_[node_a_index]->QueueUntil(
                  filter.filter->unobserved_line_end() +
                  time_estimation_buffer_seconds_);

              if (timestamp_mappers_[node_b_index] != nullptr) {
                timestamp_mappers_[node_b_index]->QueueUntil(
                    filter.filter->unobserved_line_remote_end() +
                    time_estimation_buffer_seconds_);
              }
            }
          }
        }
      }

      ++node_a_index;
    }
  }

  // Djikstra's algorithm with bit sets :)
  // The visited set.
  BitSet64 visited_set(problem.size());
  while (true) {
    // Compute the set of all nodes which are new and haven't been visited to
    // track down dependencies.
    BitSet64 new_nodes = traversed_nodes & ~visited_set;

    // And then, before changing it, record the list of nodes we've traversed as
    // now already visited.
    visited_set = traversed_nodes;

    size_t count = 0;
    for (size_t i = new_nodes.FirstBitSet(0); i < new_nodes.size();
         i = new_nodes.FirstBitSet(i + 1)) {
      for (const auto &filter : filters_per_node_[i]) {
        if (filter.filter->timestamps_size() > 0u) {
          traversed_nodes.Set(i, true);
          traversed_nodes.Set(filter.b_index, true);
        }
      }
      ++count;
    }

    if (count == 0) {
      // Finished walking the graph, see how we did.
      break;
    }
  }

  CHECK(traversed_nodes == all_live_nodes)
      << ": Found a subset of the graph which is disconnected.  This isn't "
         "solvable today, but could be with valid use case.";

  // Set any dead nodes to invalid.
  if (all_nodes != all_live_nodes) {
    const BitSet64 dead_nodes = all_nodes & (~traversed_nodes);
    for (size_t i = dead_nodes.FirstBitSet(0); i < dead_nodes.size();
         i = dead_nodes.FirstBitSet(i + 1)) {
      problem.set_live(i, false);
      VLOG(1) << "Node " << i << " is dead";
    }
    if (VLOG_IS_ON(2)) {
      problem.Debug();
    }
  }

  return problem;
}

std::tuple<std::vector<MultiNodeNoncausalOffsetEstimator::CandidateTimes>, bool>
MultiNodeNoncausalOffsetEstimator::MakeCandidateTimes() const {
  std::vector<CandidateTimes> candidate_times;
  candidate_times.resize(last_monotonics_.size());

  size_t node_a_index = 0;
  for (const auto &filters : filters_per_node_) {
    VLOG(2) << "Investigating filter for node " << node_a_index;
    BootTimestamp next_node_time = BootTimestamp::max_time();
    BootDuration next_node_duration = BootDuration::max_time();
    size_t b_index = std::numeric_limits<size_t>::max();
    NoncausalTimestampFilter *next_node_filter = nullptr;
    // Find the oldest time for each node in each filter, and solve for that
    // time.  That gives us the next timestamp for this node.
    size_t filter_index = 0;
    for (const auto &filter : filters) {
      std::optional<std::tuple<BootTimestamp, BootDuration>> candidate =
          filter.filter->Observe();

      if (candidate) {
        VLOG(2) << "Candidate for node " << node_a_index << " filter "
                << filter_index << " is " << std::get<0>(*candidate);
        if (std::get<0>(*candidate) < next_node_time) {
          next_node_time = std::get<0>(*candidate);
          next_node_duration = std::get<1>(*candidate);
          b_index = filter.b_index;
          next_node_filter = filter.filter;
        }
      }
      ++filter_index;
    }

    // We want to make sure we solve explicitly for the start time for each
    // log.  This is useless (though not all that expensive) if it is in the
    // middle of a set of data since we are just adding an extra point in the
    // middle of a line, but very useful if the start time is before any
    // points and we need to force a node to reboot.
    //
    // We can only do this meaningfully if there are data points on this node
    // before or after this node to solve for.
    const size_t next_boot = last_monotonics_[node_a_index].boot + 1;
    if (next_boot < boots_->boots[node_a_index].size() &&
        timestamp_mappers_[node_a_index] != nullptr) {
      const BootTimestamp next_start_time = BootTimestamp{
          .boot = next_boot,
          .time = timestamp_mappers_[node_a_index]->monotonic_start_time(
              next_boot)};
      // If we don't have a start time, it doesn't make sense to solve for it.
      // Ignore that case.
      if (next_start_time < next_node_time &&
          next_start_time.time != monotonic_clock::min_time) {
        VLOG(1) << "Candidate for node " << node_a_index
                << " is the next startup time, " << next_start_time;
        next_node_time = next_start_time;
        next_node_filter = nullptr;
        b_index = std::numeric_limits<size_t>::max();
        next_node_duration = BootDuration::max_time();
      }

      // We need to make sure we have solutions as well for any local messages
      // published before remote messages.  Find the oldest message for each
      // boot and make sure there's a time there.  Boots can't overlap, so if
      // we have evidence that there has been a reboot, we need to get that
      // into the interpolation function.
      const BootTimestamp next_oldest_time = BootTimestamp{
          .boot = next_boot,
          .time = timestamp_mappers_[node_a_index]->monotonic_oldest_time(
              next_boot)};
      if (next_oldest_time < next_node_time) {
        VLOG(1) << "Candidate for node " << node_a_index
                << " is the next oldest time, " << next_oldest_time
                << " not applying yet";
        next_node_time = next_oldest_time;
        next_node_filter = nullptr;
        b_index = std::numeric_limits<size_t>::max();
        next_node_duration = BootDuration::max_time();
      }
    }
    candidate_times[node_a_index] =
        CandidateTimes{.next_node_time = next_node_time,
                       .next_node_duration = next_node_duration,
                       .b_index = b_index,
                       .next_node_filter = next_node_filter};
    ++node_a_index;
  }

  // Now that we have all the candidates, confirm everything matches.
  bool boots_all_match = true;
  for (size_t i = 0; i < candidate_times.size(); ++i) {
    const CandidateTimes &candidate = candidate_times[i];
    if (candidate.next_node_time == logger::BootTimestamp::max_time()) {
      continue;
    }

    // First step, if the last solution's boot doesn't match the next solution,
    // we've got a reboot incoming and can't sort well.  Fall back to the more
    // basic exhaustive search.
    if (candidate.next_node_time.boot != last_monotonics_[i].boot) {
      boots_all_match = false;
      break;
    }

    // And then check that the other node's time also hasn't rebooted.  We might
    // not have both directions of timestamps, so this is our only clue.
    if (candidate.next_node_duration == BootDuration::max_time()) {
      continue;
    }

    DCHECK_LT(candidate.b_index, candidate_times.size());
    if (candidate_times[candidate.b_index].next_node_time.boot !=
        candidate.next_node_duration.boot) {
      boots_all_match = false;
      break;
    }
  }
  if (VLOG_IS_ON(1)) {
    LOG(INFO) << "Boots all match: " << boots_all_match;
    for (size_t i = 0; i < candidate_times.size(); ++i) {
      LOG(INFO) << "Candidate " << candidate_times[i].next_node_time
                << " duration " << candidate_times[i].next_node_duration
                << " (node " << candidate_times[i].b_index << ")";
    }
  }

  return std::make_tuple(candidate_times, boots_all_match);
}

std::optional<
    std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>>
MultiNodeNoncausalOffsetEstimator::SimultaneousSolution(
    TimestampProblem *problem,
    const std::vector<CandidateTimes> candidate_times,
    const std::vector<logger::BootTimestamp> &base_times) {
  std::vector<BootTimestamp> result_times;
  NoncausalTimestampFilter *next_filter = nullptr;
  size_t solution_index = 0;

  // Now, build up the solution points that we care about.
  size_t valid_nodes = 0;
  // We know that time advances at about 1 seconds/second.  So, a good
  // approximation for the next solution is going to be to compute the amount
  // of time that will elapse for each node to go to the points to solve, and
  // advance the minimum amount.  This should hopefully save an iteration or
  // two on the solver for minimal compute.
  chrono::nanoseconds dt{0};
  std::vector<BootTimestamp> points(problem->size(), BootTimestamp::max_time());

  for (size_t node_a_index = 0; node_a_index < candidate_times.size();
       ++node_a_index) {
    BootTimestamp next_node_time = candidate_times[node_a_index].next_node_time;
    if (next_node_time == BootTimestamp::max_time()) {
      continue;
    }
    CHECK_EQ(next_node_time.boot, base_times[node_a_index].boot);

    const chrono::nanoseconds this_dt =
        next_node_time.time - base_times[node_a_index].time;
    if (valid_nodes == 0 || this_dt < dt) {
      dt = this_dt;
    }

    ++valid_nodes;
    points[node_a_index] = next_node_time;
  }

  // Only solve if there are nodes to solve for.  Otherwise the defaults will
  // report 'no solution' which is exactly what we want.
  if (valid_nodes > 0) {
    // Apply our dt offset.
    for (size_t node_index = 0; node_index < base_times.size(); ++node_index) {
      problem->set_base_clock(node_index, {base_times[node_index].boot,
                                           base_times[node_index].time + dt});
    }
    problem->set_points(points);

    Eigen::VectorXd solution_y;
    size_t iterations;
    NewtonSolver solver;
    std::tie(solution_y, solution_index, iterations) =
        solver.SolveNewton(problem, kMaxIterations);
    std::vector<BootTimestamp> solution =
        problem->PackResults(iterations > kMaxIterations ||
                                 SOLVE_VLOG_IS_ON(solver.my_solve_number(), 2),
                             solution_y);

    if (iterations > kMaxIterations && FLAGS_crash_on_solve_failure) {
      UpdateSolution(std::move(solution));
      if (!FlushAndClose(false)) {
        return std::nullopt;
      }
      LOG(ERROR) << "Failed to converge.";
      return std::nullopt;
    }

    if (!problem->ValidateSolution(solution, true)) {
      if (!FLAGS_constrained_solve) {
        problem->ValidateSolution(solution, false);
        LOG(WARNING) << "Invalid solution, constraints not met for problem "
                     << solver.my_solve_number();
        for (size_t i = 0; i < solution.size(); ++i) {
          LOG(INFO) << "  " << solution[i];
        }
        problem->Debug();
        if (!skip_order_validation_) {
          UpdateSolution(solution);
          if (!FlushAndClose(false)) {
            return std::nullopt;
          }
          LOG(ERROR) << "Bailing, use --skip_order_validation to continue.  "
                        "Use at your own risk.";
          return std::nullopt;
        }
      } else {
        // Aw, our initial attempt to solve failed.  Now, let's try again with
        // constraints.
        for (size_t node_index = 0; node_index < base_times.size();
             ++node_index) {
          problem->set_base_clock(node_index, solution[node_index]);
        }

        if (!FLAGS_attempt_simultaneous_constrained_solve) {
          VLOG(1) << "Falling back to sequential constrained Newton.";
          return SequentialSolution(problem, candidate_times, base_times);
        }

        problem->set_points(points);
        auto solver_result =
            SolveConstrainedNewton(&solver, problem, kMaxIterations);
        if (!solver_result.has_value()) {
          return std::nullopt;
        }
        solution = std::move(std::get<0>(solver_result.value()));
        solution_y = std::move(std::get<1>(solver_result.value()));
        std::tie(std::ignore, std::ignore, solution_index, iterations) =
            *solver_result;

        if (iterations > kMaxIterations && FLAGS_crash_on_solve_failure) {
          UpdateSolution(std::move(solution));
          if (!FlushAndClose(false)) {
            return std::nullopt;
          }
          LOG(ERROR) << "Failed to converge for problem "
                     << solver.my_solve_number();
          return std::nullopt;
        }
        if (!problem->ValidateSolution(solution, false)) {
          LOG(WARNING) << "Invalid solution, constraints not met for problem "
                       << solver.my_solve_number();
          for (size_t i = 0; i < solution.size(); ++i) {
            LOG(INFO) << "  " << solution[i];
          }
          problem->Debug();
          if (!skip_order_validation_) {
            UpdateSolution(solution);
            if (!FlushAndClose(false)) {
              return std::nullopt;
            }
            LOG(ERROR) << "Bailing, use --skip_order_validation to continue.  "
                          "Use at your own risk.";
            return std::nullopt;
          }
        }
      }
    }

    result_times = std::move(solution);
    next_filter = candidate_times[solution_index].next_node_filter;
  }

  return std::make_tuple(next_filter, std::move(result_times), solution_index);
}

bool MultiNodeNoncausalOffsetEstimator::CheckInvalidDistance(
    const std::vector<BootTimestamp> &result_times,
    const std::vector<BootTimestamp> &solution) {
  // If times are close enough, drop the invalid time.
  const chrono::nanoseconds invalid_distance =
      InvalidDistance(result_times, solution);
  if (invalid_distance <= chrono::nanoseconds(FLAGS_max_invalid_distance_ns)) {
    VLOG(1) << "Times can't be compared by " << invalid_distance.count()
            << "ns";
    for (size_t i = 0; i < result_times.size(); ++i) {
      VLOG(1) << "  " << result_times[i] << " vs " << solution[i] << " -> "
              << (result_times[i].time - solution[i].time).count() << "ns";
    }
    VLOG(1) << "Ignoring because it is close enough.";
    return true;
  }
  // Somehow the new solution is better *and* worse than the old
  // solution...  This is an internal failure because that means time
  // goes backwards on a node.
  CHECK_EQ(result_times.size(), solution.size());
  LOG(INFO) << "Times can't be compared by " << invalid_distance.count()
            << "ns";
  for (size_t i = 0; i < result_times.size(); ++i) {
    LOG(INFO) << "  " << result_times[i] << " vs " << solution[i] << " -> "
              << (result_times[i].time - solution[i].time).count() << "ns";
  }

  if (skip_order_validation_) {
    LOG(ERROR) << "Skipping because --skip_order_validation";
  } else {
    UpdateSolution(solution);
    if (!FlushAndClose(false)) {
      return false;
    }
    LOG(ERROR) << "Please investigate.  Use --max_invalid_distance_ns="
               << invalid_distance.count() << " to ignore this.";
    return false;
  }
  return true;
}

std::optional<
    std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>>
MultiNodeNoncausalOffsetEstimator::SequentialSolution(
    TimestampProblem *problem,
    const std::vector<CandidateTimes> candidate_times,
    const std::vector<logger::BootTimestamp> &base_times) {
  std::vector<BootTimestamp> result_times;
  NoncausalTimestampFilter *next_filter = nullptr;
  size_t solution_index = 0;

  for (size_t node_a_index = 0; node_a_index < candidate_times.size();
       ++node_a_index) {
    VLOG(2) << "Investigating filter for node " << node_a_index;
    BootTimestamp next_node_time = candidate_times[node_a_index].next_node_time;
    BootDuration next_node_duration =
        candidate_times[node_a_index].next_node_duration;
    NoncausalTimestampFilter *next_node_filter =
        candidate_times[node_a_index].next_node_filter;
    size_t b_index = candidate_times[node_a_index].b_index;
    if (next_node_time == BootTimestamp::max_time()) {
      continue;
    }

    if (next_node_filter != nullptr) {
      VLOG(2) << "Trying " << next_node_time << " " << next_node_duration
              << " for node " << node_a_index;
    } else {
      VLOG(1) << "Trying " << next_node_time << " for node " << node_a_index;
    }

    // TODO(austin): If we start supporting only having 1 direction of
    // timestamps, we might need to change our assumptions around
    // BootTimestamp and BootDuration.
    bool boots_match = next_node_time.boot == base_times[node_a_index].boot;

    // Make sure the paired time also has a matching boot.
    if (next_node_duration != BootDuration::max_time()) {
      if (next_node_duration.boot != base_times[b_index].boot) {
        boots_match = false;
      }
    }

    if (boots_match) {
      // Optimize, and save the time into times if earlier than time.
      for (size_t node_index = 0; node_index < base_times.size();
           ++node_index) {
        // Offset everything based on the elapsed time since the last solution
        // on the node we are solving for.  The rate that time elapses should
        // be ~1.
        problem->set_base_clock(
            node_index,
            {base_times[node_index].boot,
             base_times[node_index].time +
                 (next_node_time.time - base_times[node_a_index].time)});
      }
    } else {
      // Otherwise just pick the base time from before to try.
      for (size_t node_index = 0; node_index < base_times.size();
           ++node_index) {
        problem->set_base_clock(node_index, base_times[node_index]);
      }
      // And we know our solution node will have the wrong boot, so replace
      // it entirely.
      problem->set_base_clock(node_a_index, next_node_time);

      // And update the paired boot for the paired node.
      if (next_node_duration != BootDuration::max_time()) {
        if (next_node_duration.boot != base_times[b_index].boot) {
          problem->set_base_clock(
              b_index, BootTimestamp{.boot = next_node_duration.boot,
                                     .time = next_node_time.time +
                                             next_node_duration.duration});
        }
      }
    }

    if (!problem->HasObservations(node_a_index)) {
      VLOG(1) << "No observations, checking if there's a filter";
      CHECK(next_node_filter == nullptr)
          << ": No observations, but this isn't a start time.";
      continue;
    }

    std::vector<BootTimestamp> points(problem->size(),
                                      BootTimestamp::max_time());
    if (VLOG_IS_ON(2)) {
      problem->Debug();
    }
    points[node_a_index] = next_node_time;

    problem->set_points(points);

    NewtonSolver solver;
    Eigen::VectorXd solution_y;
    size_t iterations;
    size_t my_solution_index;
    std::tie(solution_y, my_solution_index, iterations) =
        solver.SolveNewton(problem, kMaxIterations);
    std::vector<BootTimestamp> solution =
        problem->PackResults(iterations > kMaxIterations ||
                                 SOLVE_VLOG_IS_ON(solver.my_solve_number(), 2),
                             solution_y);

    if (iterations > kMaxIterations && FLAGS_crash_on_solve_failure) {
      UpdateSolution(std::move(solution));
      if (!FlushAndClose(false)) {
        return std::nullopt;
      }
      LOG(ERROR) << "Failed to converge for problem "
                 << solver.my_solve_number();
      return std::nullopt;
    }

    // Bypass checking if order validation is turned off.  This lets us dump a
    // CSV file so we can view the problem and figure out what to do.  The
    // results won't make sense.
    if (!problem->ValidateSolution(solution, true)) {
      if (!FLAGS_constrained_solve) {
        // Do it non-quiet now.
        problem->ValidateSolution(solution, false);

        LOG(WARNING) << "Invalid solution, constraints not met for problem "
                     << solver.my_solve_number();
        for (size_t i = 0; i < solution.size(); ++i) {
          LOG(INFO) << "  " << solution[i];
        }
        problem->Debug();
        if (!skip_order_validation_) {
          UpdateSolution(solution);
          if (!FlushAndClose(false)) {
            return std::nullopt;
          }
          LOG(ERROR) << "Bailing, use --skip_order_validation to continue.  "
                        "Use at your own risk.";
          return std::nullopt;
        }
      } else {
        // Seed with our last solution.
        for (size_t node_index = 0; node_index < base_times.size();
             ++node_index) {
          problem->set_base_clock(node_index, solution[node_index]);
        }
        // And now resolve constrained.
        problem->set_points(points);
        auto solver_result =
            SolveConstrainedNewton(&solver, problem, kMaxIterations);
        if (!solver_result.has_value()) {
          return std::nullopt;
        }
        solution = std::move(std::get<0>(solver_result.value()));
        solution_y = std::move(std::get<1>(solver_result.value()));
        std::tie(std::ignore, std::ignore, solution_index, iterations) =
            *solver_result;

        if (iterations > kMaxIterations && FLAGS_crash_on_solve_failure) {
          UpdateSolution(std::move(solution));
          if (!FlushAndClose(false)) {
            return std::nullopt;
          }
          LOG(ERROR) << "Failed to converge for problem "
                     << solver.my_solve_number();
          return std::nullopt;
        }
        if (!problem->ValidateSolution(solution, false)) {
          LOG(WARNING) << "Invalid solution, constraints not met for problem "
                       << solver.my_solve_number();
          for (size_t i = 0; i < solution.size(); ++i) {
            LOG(INFO) << "  " << solution[i];
          }
          problem->Debug();
          if (!skip_order_validation_) {
            UpdateSolution(solution);
            if (!FlushAndClose(false)) {
              return std::nullopt;
            }
            LOG(ERROR) << "Bailing, use --skip_order_validation to continue.  "
                          "Use at your own risk.";
            return std::nullopt;
          }
        }
      }
    }

    if (VLOG_IS_ON(1)) {
      VLOG(1) << "Candidate solution for node " << node_a_index << " is";
      for (size_t i = 0; i < solution.size(); ++i) {
        VLOG(1) << "  " << solution[i];
      }
    }

    if (result_times.empty()) {
      // This is the first solution candidate, so don't bother comparing.
      result_times = std::move(solution);
      next_filter = next_node_filter;
      solution_index = node_a_index;
      continue;
    }

    switch (CompareTimes(result_times, solution)) {
      // The old solution is before or at the new solution.  This means that
      // the old solution is a better result, so ignore this one.
      case TimeComparison::kBefore:
      case TimeComparison::kEq:
        break;
      case TimeComparison::kAfter:
        // The new solution is better!  Save it.
        result_times = std::move(solution);
        next_filter = next_node_filter;
        solution_index = node_a_index;
        break;
      case TimeComparison::kInvalid: {
        if (!CheckInvalidDistance(result_times, solution)) {
          return std::nullopt;
        }
        if (next_node_filter) {
          std::optional<std::tuple<logger::BootTimestamp, logger::BootDuration>>
              result = next_node_filter->Consume();
          CHECK(result);
          WriteFilter(next_node_filter, *result);

          // We shouldn't pop since we don't know if this is the oldest one or
          // not, and this won't happen often.  Worst case, we leave 1 timestamp
          // queued up that we don't need.
          //
          // We could get more clever and only pop it if it "matches" the
          // eventual result, but I suspect the code to do that will be more
          // expensive than tackling it when it is actually time to pop it given
          // the frequency of the issue.
        }
      } break;
    }
  }

  return std::make_tuple(next_filter, std::move(result_times), solution_index);
}

std::optional<
    std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>>
MultiNodeNoncausalOffsetEstimator::NextSolution(
    TimestampProblem *problem, const std::vector<BootTimestamp> &base_times) {
  // Ok, now solve for the minimum time on each channel.
  bool boots_all_match = true;
  std::vector<CandidateTimes> candidate_times;
  std::tie(candidate_times, boots_all_match) = MakeCandidateTimes();

  std::optional<
      std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>>
      result;

  // We can significantly speed things up if we know that all the boots match by
  // solving for everything at once.  If the boots don't match, the combined min
  // that happens inside the solver doesn't make a lot of sense since we are
  // actually using the boot from the candidate times to figure out which
  // interpolation function to use under the hood.
  if (boots_all_match) {
    result =
        SimultaneousSolution(problem, std::move(candidate_times), base_times);
  } else {
    // If all the boots don't match, fall back to the old method of comparing
    // all the solutions individually.
    result =
        SequentialSolution(problem, std::move(candidate_times), base_times);
  }
  if (VLOG_IS_ON(1) && result.has_value()) {
    VLOG(1) << "Best solution is for node " << std::get<2>(*result);
    for (size_t i = 0; i < std::get<1>(*result).size(); ++i) {
      VLOG(1) << "  " << std::get<1>(*result)[i];
    }
  }
  return result;
}

void MultiNodeNoncausalOffsetEstimator::UpdateSolution(
    std::vector<BootTimestamp> result_times) {
  // Now, figure out what distributed should be.  It should move at the rate of
  // the max elapsed time so that conversions to and from it don't round to bad
  // values.
  const chrono::nanoseconds dt = MaxElapsedTime(last_monotonics_, result_times);
  last_distributed_ += dt;
  for (size_t i = 0; i < result_times.size(); ++i) {
    if (result_times[i] == BootTimestamp::min_time()) {
      // Found an unknown node.  Move its time along by the amount the
      // distributed clock moved.
      result_times[i] = last_monotonics_[i] + dt;
    }
  }
  last_monotonics_ = std::move(result_times);

  if (fp_) {
    fprintf(
        fp_, "%.9f",
        chrono::duration<double>(last_distributed_.time_since_epoch()).count());
    for (const BootTimestamp t : last_monotonics_) {
      fprintf(fp_, ", %.9f",
              chrono::duration<double>(t.time.time_since_epoch()).count());
    }
    fprintf(fp_, "\n");
  }
}

void MultiNodeNoncausalOffsetEstimator::WriteFilter(
    NoncausalTimestampFilter *next_filter,
    std::tuple<logger::BootTimestamp, logger::BootDuration> sample) {
  if (filter_fps_.size() > 0 && next_filter) {
    const int node_a_index =
        configuration::GetNodeIndex(configuration(), next_filter->node_a());
    const int node_b_index =
        configuration::GetNodeIndex(configuration(), next_filter->node_b());

    FILE *fp = filter_fps_[node_a_index][node_b_index];
    if (fp == nullptr) {
      fp = filter_fps_[node_a_index][node_b_index] =
          fopen(CsvPath("timestamp_noncausal_",
                        next_filter->node_a()->name()->string_view(), "_",
                        next_filter->node_b()->name()->string_view(), ".csv")
                    .c_str(),
                "w");
      fprintf(fp, "time_since_start,sample_ns,filtered_offset\n");
    }

    if (last_monotonics_[node_a_index].boot == std::get<0>(sample).boot) {
      fprintf(
          fp, "%.9f, %.9f, %.9f\n",
          std::chrono::duration_cast<std::chrono::duration<double>>(
              last_distributed_.time_since_epoch() + std::get<0>(sample).time -
              last_monotonics_[node_a_index].time)
              .count(),
          std::chrono::duration_cast<std::chrono::duration<double>>(
              std::get<0>(sample).time.time_since_epoch())
              .count(),
          std::chrono::duration_cast<std::chrono::duration<double>>(
              std::get<1>(sample).duration)
              .count());
    } else {
      LOG(WARNING) << "Not writing point, missmatched boot.";
    }
  }
}

std::optional<std::optional<
    std::tuple<distributed_clock::time_point, std::vector<BootTimestamp>>>>
MultiNodeNoncausalOffsetEstimator::NextTimestamp() {
  // TODO(austin): Detect and handle there being fewer nodes in the log file
  // than in replay, or them being in a different order.
  TimestampProblem problem = MakeProblem();

  // Ok, now solve for the minimum time on each channel.
  auto next_solution = NextSolution(&problem, last_monotonics_);
  if (!next_solution.has_value()) {
    return std::nullopt;
  }
  std::vector<BootTimestamp> result_times =
      std::move(std::get<1>(next_solution.value()));
  NoncausalTimestampFilter *next_filter = nullptr;
  int solution_node_index = 0;
  std::tie(next_filter, std::ignore, solution_node_index) =
      next_solution.value();

  CHECK(!all_done_);

  // All done.
  if (result_times.empty()) {
    if (first_solution_) {
      VLOG(1) << "No more timestamps and the first solution.";
      // If this is our first time, there is no solution.  Instead of giving up
      // completely, (and providing no estimate of time at all), just say that
      // everything is on the distributed clock.  This will then get used as a
      // 1:1 mapping.
      first_solution_ = false;
      if (fp_) {
        fprintf(fp_, "0.000000000");
        for (size_t i = 0; i < NodesCount(); ++i) {
          fprintf(fp_, ", 0.000000000");
        }
        fprintf(fp_, "\n");
      }
      return std::make_tuple(
          distributed_clock::epoch(),
          std::vector<BootTimestamp>(NodesCount(), BootTimestamp::epoch()));
    }
    if (VLOG_IS_ON(1)) {
      LOG(INFO) << "Found no more timestamps.";
      for (const auto &filters : filters_per_node_) {
        for (const auto &filter : filters) {
          filter.filter->Debug();
        }
      }
    }
    all_done_ = true;

    // TODO(austin): Instead of giving up forever, give up as far as we can look
    // into the future.  This would let nodes start up unknown and converge to
    // something useful when they connect.
    if (fp_) {
      fflush(fp_);
    }
    std::optional<std::optional<
        std::tuple<distributed_clock::time_point, std::vector<BootTimestamp>>>>
        result;
    result.emplace(std::nullopt);
    CHECK(result.has_value());
    CHECK(!result.value().has_value());
    return result;
  }

  {
    size_t index = 0;
    for (auto t : result_times) {
      VLOG(1)
          << "Time: " << t << " "
          << logged_configuration_->nodes()->Get(index)->name()->string_view();
      ++index;
    }
  }

  std::tuple<logger::BootTimestamp, logger::BootDuration> sample;
  if (first_solution_) {
    first_solution_ = false;

    // Force any unknown nodes to track the distributed clock (which starts at 0
    // too).
    for (BootTimestamp &time : result_times) {
      if (time == BootTimestamp::min_time()) {
        time = BootTimestamp::epoch();
      }
    }
    if (next_filter) {
      // This isn't a start time because we have a corresponding filter.
      sample = *next_filter->Consume();
      VLOG(1) << "Sample is " << std::get<0>(sample) << " from "
              << next_filter->node_a()->name()->string_view();
      next_filter->Pop(std::get<0>(sample) - time_estimation_buffer_seconds_);
    }
  } else {
    if (next_filter) {
      // This isn't a start time because we have a corresponding filter.
      sample = *next_filter->Consume();
      VLOG(1) << "Sample is " << std::get<0>(sample) << " from "
              << next_filter->node_a()->name()->string_view();
      next_filter->Pop(std::get<0>(sample) - time_estimation_buffer_seconds_);
    }
    // We found a good sample, so consume it.  If it is a duplicate, we still
    // want to consume it.  But, if this is the first time around, we want to
    // re-solve by recursing (once) to pickup the better base.

    TimeComparison compare = CompareTimes(last_monotonics_, result_times);
    switch (compare) {
      case TimeComparison::kBefore:
        break;
      case TimeComparison::kAfter:
        problem.Debug();
        for (size_t i = 0; i < result_times.size(); ++i) {
          LOG(INFO) << "  " << last_monotonics_[i] << " vs " << result_times[i]
                    << " -> "
                    << (last_monotonics_[i].time - result_times[i].time).count()
                    << "ns";
        }
        UpdateSolution(std::move(result_times));
        WriteFilter(next_filter, sample);
        if (!FlushAndClose(false)) {
          return std::nullopt;
        }
        LOG(ERROR)
            << "Found a solution before the last returned solution on node "
            << solution_node_index;
        return std::nullopt;
      case TimeComparison::kEq:
        WriteFilter(next_filter, sample);
        return NextTimestamp();
      case TimeComparison::kInvalid: {
        const chrono::nanoseconds invalid_distance =
            InvalidDistance(last_monotonics_, result_times);
        if (invalid_distance <=
            chrono::nanoseconds(FLAGS_max_invalid_distance_ns)) {
          WriteFilter(next_filter, sample);
          return NextTimestamp();
        }
        LOG(INFO) << "Times can't be compared by " << invalid_distance.count()
                  << "ns";
        CHECK_EQ(last_monotonics_.size(), result_times.size());
        for (size_t i = 0; i < result_times.size(); ++i) {
          LOG(INFO) << "  " << last_monotonics_[i] << " vs " << result_times[i]
                    << " -> "
                    << (last_monotonics_[i].time - result_times[i].time).count()
                    << "ns";
        }
        UpdateSolution(std::move(result_times));
        WriteFilter(next_filter, sample);
        if (!FlushAndClose(false)) {
          return std::nullopt;
        }
        LOG(ERROR) << "Please investigate.  Use --max_invalid_distance_ns="
                   << invalid_distance.count() << " to ignore this.";
        return std::nullopt;
      } break;
    }
  }

  UpdateSolution(std::move(result_times));
  WriteFilter(next_filter, sample);

  // And freeze everything.
  {
    size_t node_index = 0;
    for (const auto &filters : filters_per_node_) {
      for (const auto &filter : filters) {
        filter.filter->FreezeUntil(last_monotonics_[node_index],
                                   last_monotonics_[filter.b_index]);
      }
      ++node_index;
    }
  }

  FlushAllSamples(false);
  return std::make_tuple(last_distributed_, last_monotonics_);
}

void MultiNodeNoncausalOffsetEstimator::FlushAllSamples(bool finish) {
  size_t node_index = 0;
  for (NodeSamples &node_samples : node_samples_) {
    size_t sending_node_index = 0;
    for (SingleNodeSamples &samples : node_samples.nodes) {
      if (samples.messages.size() == 0) {
        ++sending_node_index;
        continue;
      }

      FILE *samples_fp = sample_fps_[node_index][sending_node_index];
      if (samples_fp == nullptr) {
        samples_fp = sample_fps_[node_index][sending_node_index] =
            fopen(CsvPath("timestamp_noncausal_",
                          logged_configuration()
                              ->nodes()
                              ->Get(node_index)
                              ->name()
                              ->string_view(),
                          "_",
                          logged_configuration()
                              ->nodes()
                              ->Get(sending_node_index)
                              ->name()
                              ->string_view(),
                          "_samples.csv")
                      .c_str(),
                  "w");
        fprintf(samples_fp,
                "time_since_start,sample_ns,monotonic,monotonic+offset("
                "remote)\n");
      }

      auto times_it = times_.begin();
      while (!samples.messages.empty() && times_it != times_.end()) {
        const std::pair<BootTimestamp, BootTimestamp> &message =
            *samples.messages.begin();
        auto next = times_it + 1;
        while (next != times_.end()) {
          if (std::get<1>(*next)[node_index] < message.first) {
            times_it = next;
            next = times_it + 1;
          } else {
            break;
          }
        }

        distributed_clock::time_point distributed;
        const distributed_clock::time_point d0 = std::get<0>(*times_it);
        const BootTimestamp t0 = std::get<1>(*times_it)[node_index];
        if (next == times_.end()) {
          if (!finish) {
            break;
          }
          CHECK_EQ(t0.boot, message.first.boot);
          distributed = message.first.time - t0.time + d0;
        } else {
          const distributed_clock::time_point d1 = std::get<0>(*next);
          const BootTimestamp t1 = std::get<1>(*next)[node_index];
          if (t0.boot == t1.boot) {
            distributed = ::aos::message_bridge::ToDistributedClock(
                d0, d1, t0.time, t1.time, message.first.time);
          } else if (t0.boot == message.first.boot) {
            distributed = message.first.time - t0.time + d0;
          } else if (t1.boot == message.first.boot) {
            distributed = message.first.time - t1.time + d1;
          } else {
            LOG(FATAL) << "Boots don't match";
          }
        }
        fprintf(samples_fp, "%.9f, %.9f, %.9f, %.9f\n",
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    distributed.time_since_epoch())
                    .count(),
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    message.second.time - message.first.time)
                    .count(),
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    message.first.time.time_since_epoch())
                    .count(),
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    message.second.time.time_since_epoch())
                    .count());

        samples.messages.erase(samples.messages.begin());
      }
      ++sending_node_index;
    }
    ++node_index;
  }
}

}  // namespace message_bridge
}  // namespace aos
