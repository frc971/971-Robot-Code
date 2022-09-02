#include "aos/network/multinode_timestamp_filter.h"

#include <chrono>
#include <functional>
#include <map>

#include "absl/strings/str_join.h"
#include "aos/configuration.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "glog/logging.h"

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

#define SOLVE_VLOG_IS_ON(v)                                               \
  (VLOG_IS_ON(v) ||                                                       \
   (static_cast<int32_t>(my_solve_number_) == FLAGS_debug_solve_number && \
    FLAGS_debug_solve_number != -1))

#define SOLVE_VLOG(v) LOG_IF(INFO, SOLVE_VLOG_IS_ON(v))

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;
using aos::logger::BootDuration;
using aos::logger::BootTimestamp;

const Eigen::IOFormat kHeavyFormat(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                   ", ", ";\n", "[", "]", "[", "]");

template <class... Args>
std::string CsvPath(Args &&...args) {
  return absl::StrCat(FLAGS_timestamp_csv_folder, "/",
                      std::forward<Args>(args)...);
}

}  // namespace

size_t TimestampProblem::solve_number_ = 0u;

TimestampProblem::TimestampProblem(size_t count) {
  CHECK_GT(count, 1u);
  clock_offset_filter_for_node_.resize(count);
  base_clock_.resize(count);
  live_.resize(count, true);
  node_mapping_.resize(count, 0);
  my_solve_number_ = solve_number_++;
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
    if (!filter.filter->timestamps_empty(
            base_clock_[node_a].boot, base_clock_[filter.b_index].boot)) {
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
      if (filter.filter->timestamps_empty(
              base_clock_[i].boot, base_clock_[filter.b_index].boot)) {
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
          solution[filter.b_index], quiet);
      if (!iteration) {
        filter.filter->ValidateSolution(filter.b_filter, filter.pointer,
                                        solution[i], 0.0,
                                        solution[filter.b_index], 0.0, quiet);
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

TimestampProblem::Derivitives TimestampProblem::ComputeDerivitives(
      const Eigen::Ref<Eigen::VectorXd> time_offsets) {
  Derivitives result;

  // We get back both interger and double remainders for the gradient.  We then
  // add them all up.  Rather than doing that purely as doubles, let's save up
  // both, compute the result, then convert the remainder to doubles.  This is a
  // bigger issue at the start when we are extrapolating a lot, and the offsets
  // can be quite large in each direction.
  Eigen::Matrix<chrono::nanoseconds, Eigen::Dynamic, 1> intgrad =
      Eigen::Matrix<chrono::nanoseconds, Eigen::Dynamic, 1>::Zero(live_nodes_);
  result.gradient = Eigen::VectorXd::Zero(live_nodes_);

  result.hessian = Eigen::MatrixXd::Zero(live_nodes_, live_nodes_);

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
      filter.pointer = std::move(offset_error.first);

      const std::pair<chrono::nanoseconds, double> error =
          std::make_pair(std::get<0>(offset_error.second),
                         std::get<1>(offset_error.second) - kMinNetworkDelay);

      std::pair<chrono::nanoseconds, double> grad;
      double hess;

      grad = std::make_pair(2 * error.first, 2 * error.second);
      hess = 2.0;

      intgrad(a_solution_index) += -grad.first;
      intgrad(b_solution_index) += grad.first;
      result.gradient(a_solution_index) += -grad.second;
      result.gradient(b_solution_index) += grad.second;

      VLOG(2) << "  Filter pair "
              << filter.filter->node_a()->name()->string_view() << "("
              << a_solution_index << ") -> "
              << filter.filter->node_b()->name()->string_view() << "("
              << b_solution_index << "): " << std::setprecision(12)
              << error.first.count() << " + " << error.second;

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
    }
  }

  for (int i = 0; i < intgrad.rows(); ++i) {
    result.gradient(i) += static_cast<double>(intgrad(i).count());
  }

  return result;
}

std::tuple<Eigen::VectorXd, size_t> TimestampProblem::Newton(
    const Eigen::Ref<Eigen::VectorXd> time_offsets,
    const std::vector<logger::BootTimestamp> &points) {
  CHECK_GT(live_nodes_, 0u) << ": No live nodes to solve for.";
  const Derivitives derivitives = ComputeDerivitives(time_offsets);

  const Eigen::MatrixXd constraint_jacobian =
      Eigen::MatrixXd::Ones(1, live_nodes_) / static_cast<double>(live_nodes_);
  // https://www.cs.purdue.edu/homes/jhonorio/16spring-cs52000-equality.pdf
  //
  // Queue long explanation for why this is the right math...
  //
  // Our cost function is piecewise quadratic and simple by design.  It should
  // also be convex.  This means it is equivalent for us to drive the gradient
  // to 0 to find the corresponding times on all nodes.
  //
  // This gets us close but doesn't let us solve for the time corresponding to a
  // specific time on one node on all the nodes.
  //
  // To do this, we want a Newton solver which works for equality constraints.
  //   argmin f(x) subject to {A x = b}
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

  Eigen::MatrixXd a;
  a.resize(live_nodes_ + 1, live_nodes_ + 1);
  a.block(0, 0, live_nodes_, live_nodes_) = derivitives.hessian;
  a.block(0, live_nodes_, live_nodes_, 1) = constraint_jacobian.transpose();
  a.block(live_nodes_, 0, 1, live_nodes_) = constraint_jacobian;
  a(live_nodes_, live_nodes_) = 0.0;

  Eigen::VectorXd b = Eigen::VectorXd::Zero(live_nodes_ + 1);
  b.block(0, 0, live_nodes_, 1) = -derivitives.gradient;

  // Now, we want to set b(live_nodes_) to be -time_offset for the earliest
  // clock.
  //
  // To save ourselves a fair amount of compute, we can take the min here.  That
  // will drive us back the furthest back in time for all provided nodes without
  // having to solve N times and look for the earliest solution.
  size_t solution_node = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i] == logger::BootTimestamp::max_time()) {
      continue;
    }

    CHECK_EQ(points[i].boot, base_clock(i).boot);
    const double candidate_b =
        chrono::duration<double, std::nano>(points[i].time - base_clock(i).time)
            .count() -
        time_offsets(NodeToFullSolutionIndex(i));
    if (candidate_b < b(live_nodes_) ||
        solution_node == std::numeric_limits<size_t>::max()) {
      VLOG(2) << "Node " << i << ", solution time " << points[i]
              << ", base_clock " << base_clock(i) << ", error " << candidate_b
              << " time offset " << time_offsets(NodeToFullSolutionIndex(i));
      b(live_nodes_) = candidate_b;
      solution_node = i;
    }
  }

  CHECK_NE(solution_node, std::numeric_limits<size_t>::max())
      << ": No solution nodes, please investigate";

  return std::tuple<Eigen::VectorXd, size_t>(a.colPivHouseholderQr().solve(b),
                                             solution_node);
}

std::tuple<std::vector<BootTimestamp>, size_t, size_t>
TimestampProblem::SolveNewton(const std::vector<logger::BootTimestamp> &points,
                              const size_t max_iterations) {
  MaybeUpdateNodeMapping();
  SOLVE_VLOG(2) << "Starting to solve problem number " << my_solve_number_;
  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i] != logger::BootTimestamp::max_time()) {
      SOLVE_VLOG(2) << "Solving for node " << i << " at " << points[i];
    }
  }
  Eigen::VectorXd data = Eigen::VectorXd::Zero(live_nodes_);

  size_t iteration = 0;
  size_t solution_node;
  while (true) {
    Eigen::VectorXd step;
    std::tie(step, solution_node) = Newton(data, points);

    if (VLOG_IS_ON(2)) {
      // Print out the gradient ignoring the component removed by the equality
      // constraint.  This tells us what gradient we are depending to try to
      // finish our solution.
      const Eigen::MatrixXd constraint_jacobian =
          Eigen::MatrixXd::Ones(1, live_nodes_) /
          static_cast<double>(live_nodes_);
      Eigen::VectorXd adjusted_grad =
          ComputeDerivitives(data).gradient +
          step(live_nodes_) * constraint_jacobian.transpose();

      VLOG(2) << "Adjusted grad " << iteration << " -> "
              << std::setprecision(12) << std::fixed << std::setfill(' ')
              << adjusted_grad.transpose().format(kHeavyFormat);
    }

    VLOG(2) << "Step " << iteration << " -> " << std::setprecision(12)
            << std::fixed << std::setfill(' ')
            << step.transpose().format(kHeavyFormat);
    // We got there if the max step is small (this is strongly correlated to the
    // gradient since the Hessian is constant), and our solution node's time is
    // also close.
    if (step.block(0, 0, live_nodes_, 1).lpNorm<Eigen::Infinity>() < 1e-4 &&
        std::abs(
            chrono::duration<double, std::nano>(points[solution_node].time -
                                                base_clock(solution_node).time)
                .count() -
            data(NodeToFullSolutionIndex(solution_node))) < 1e-4) {
      break;
    }

    data += step.block(0, 0, live_nodes_, 1);

    ++iteration;

    // We are doing all our math with both an int64 base and a double offset.
    // This lets us handle large offsets while retaining precision down to the
    // nanosecond easily.
    //
    // Some problems start out with a poor initial solution.  This is especially
    // true for the first solution.  Because we control the solver, as we
    // determine that the double is getting too big, we can move that
    // information to the int64 base clock.  Threshold this to not be *too* big
    // since it makes it hard to debug as the data keeps jumping around.
    for (size_t j = 0; j < size(); ++j) {
      const size_t solution_index = NodeToFullSolutionIndex(j);
      if (live(j) && std::abs(data(solution_index)) > 1000) {
        int64_t dsolution =
            static_cast<int64_t>(std::round(data(solution_index)));
        base_clock_[j].time += chrono::nanoseconds(dsolution);
        data(solution_index) -= dsolution;
      }
      if (live(j)) {
        SOLVE_VLOG(2) << "    live  "
                      << base_clock_[j].time +
                             std::chrono::nanoseconds(static_cast<int64_t>(
                                 std::round(data(NodeToFullSolutionIndex(j)))))
                      << " "
                      << (data(NodeToFullSolutionIndex(j)) -
                          std::round(data(NodeToFullSolutionIndex(j))))
                      << " (unrounded: " << data(NodeToFullSolutionIndex(j))
                      << ")";
      } else {
        SOLVE_VLOG(2) << "    dead  " << aos::monotonic_clock::min_time;
      }
    }

    // And finally, don't let us iterate forever.  If it isn't converging,
    // report back.
    if (iteration > max_iterations) {
      break;
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i] != logger::BootTimestamp::max_time()) {
      SOLVE_VLOG(2) << "Solving for node " << i << " of " << points[i] << " in "
                    << iteration << " cycles";
    }
  }
  std::vector<BootTimestamp> result(size());
  for (size_t i = 0; i < size(); ++i) {
    if (live(i)) {
      result[i].boot = base_clock(i).boot;
      result[i].time = base_clock(i).time +
                       std::chrono::nanoseconds(static_cast<int64_t>(
                           std::round(data(NodeToFullSolutionIndex(i)))));
      if (VLOG_IS_ON(2) || iteration > max_iterations) {
        LOG(INFO) << "live  " << result[i] << " "
                  << (data(NodeToFullSolutionIndex(i)) -
                      std::round(data(NodeToFullSolutionIndex(i))))
                  << " (unrounded: " << data(NodeToFullSolutionIndex(i)) << ")";
      }
    } else {
      result[i] = BootTimestamp::min_time();
      if (VLOG_IS_ON(2) || iteration > max_iterations) {
        LOG(INFO) << "dead  " << result[i];
      }
    }
  }
  if (iteration > max_iterations) {
    LOG(ERROR) << "Failed to converge.";
  }

  return std::make_tuple(std::move(result), solution_node, iteration);
}

void TimestampProblem::MaybeUpdateNodeMapping() {
  if (node_mapping_valid_) {
    return;
  }
  size_t live_node_index = 0;
  for (size_t i = 0; i < node_mapping_.size(); ++i) {
    if (live(i)) {
      node_mapping_[i] = live_node_index;
      ++live_node_index;
    } else {
      node_mapping_[i] = std::numeric_limits<size_t>::max();
    }
  }
  live_nodes_ = live_node_index;
  live_constraints_ = LiveConstraintsCount();
  node_mapping_valid_ = true;

  // Reset the cached state each time we solve.
  for (size_t i = 0u; i < clock_offset_filter_for_node_.size(); ++i) {
    for (struct FilterPair &filter : clock_offset_filter_for_node_[i]) {
      filter.pointer = NoncausalTimestampFilter::Pointer();
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
        gradients[i].emplace_back(
            std::string("- ") +
            filter.filter->DebugOffsetError(
                filter.b_filter, NoncausalTimestampFilter::Pointer(),
                base_clock_[i], 0.0, base_clock_[filter.b_index], 0.0, i,
                filter.b_index));
        gradients[filter.b_index].emplace_back(filter.filter->DebugOffsetError(
            filter.b_filter, NoncausalTimestampFilter::Pointer(),
            base_clock_[i], 0.0, base_clock_[filter.b_index], 0.0, i,
            filter.b_index));
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

std::optional<const std::tuple<distributed_clock::time_point,
                               std::vector<BootTimestamp>> *>
InterpolatedTimeConverter::QueueNextTimestamp() {
  std::optional<
      std::tuple<distributed_clock::time_point, std::vector<BootTimestamp>>>
      next_time = NextTimestamp();
  if (!next_time) {
    VLOG(1) << "Last timestamp, calling it quits";
    at_end_ = true;
    return std::nullopt;
  }

  VLOG(1) << "Fetched next timestamp while solving: " << std::get<0>(*next_time)
          << " ->";
  for (BootTimestamp t : std::get<1>(*next_time)) {
    VLOG(1) << "  " << t;
  }

  CHECK_EQ(node_count_, std::get<1>(*next_time).size());

  if (times_.empty()) {
    for (BootTimestamp t : std::get<1>(*next_time)) {
      CHECK_EQ(t.boot, 0u);
    }
  } else {
    bool rebooted = false;
    for (size_t i = 0; i < node_count_; ++i) {
      if (std::get<1>(times_.back())[i].boot !=
          std::get<1>(*next_time)[i].boot) {
        rebooted = true;
        break;
      }
    }
    if (rebooted) {
      CHECK(reboot_found_);
      reboot_found_(std::get<0>(*next_time), std::get<1>(*next_time));
    }
  }
  times_.emplace_back(std::move(*next_time));
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

  const distributed_clock::time_point d0 =  std::get<0>(p0);
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

void MultiNodeNoncausalOffsetEstimator::FlushAndClose(bool destructor) {
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
  }
  if (sample_fps_.size() != 0) {
    for (std::vector<FILE *> &filter_fp : sample_fps_) {
      for (FILE *&fp : filter_fp) {
        if (fp != nullptr) {
          fclose(fp);
        }
      }
    }
  }

  if (all_done_) {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      for (const auto &filter : filters) {
        std::optional<std::tuple<BootTimestamp, BootDuration>> next =
            filter.filter->Consume();
        if (next) {
          skip_order_validation_
              ? LOG(WARNING)
              : LOG(FATAL) << "MultiNodeNoncausalOffsetEstimator reported all "
                              "done, but "
                           << node_a_index << " -> " << filter.b_index
                           << " found more data at time " << std::get<0>(*next)
                           << ".  Time estimation was silently wrong.";
        }
      }
      ++node_a_index;
    }
  }

  // Make sure everything is flushed to disk.
  if (!node_samples_.empty()) {
    for (NodeSamples &node : node_samples_) {
      for (SingleNodeSamples &timestamps : node.nodes) {
        CHECK(timestamps.messages.empty());
      }
    }
  }
}

MultiNodeNoncausalOffsetEstimator::~MultiNodeNoncausalOffsetEstimator() {
  FlushAndClose(true);
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

  for (size_t node_index = 0; node_index < timestamp_mappers_.size();
       ++node_index) {
    if (timestamp_mappers_[node_index] != nullptr) {
      // Make sure we have enough data queued such that if we are going to
      // have a timestamp on this filter, we do have a timestamp queued.
      timestamp_mappers_[node_index]->QueueFor(time_estimation_buffer_seconds_);
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

std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>
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
    std::tuple<std::vector<BootTimestamp>, size_t, size_t> solution =
        problem->SolveNewton(points, kMaxIterations);

    if (std::get<2>(solution) > kMaxIterations) {
      UpdateSolution(std::move(std::get<0>(solution)));
      FlushAndClose(false);
      LOG(FATAL) << "Failed to converge.";
    }

    if (!problem->ValidateSolution(std::get<0>(solution), false)) {
      LOG(WARNING) << "Invalid solution, constraints not met.";
      for (size_t i = 0; i < std::get<0>(solution).size(); ++i) {
        LOG(INFO) << "  " << std::get<0>(solution)[i];
      }
      problem->Debug();
      if (!skip_order_validation_) {
        UpdateSolution(std::move(std::get<0>(solution)));
        FlushAndClose(false);
        LOG(FATAL) << "Bailing, use --skip_order_validation to continue.  "
                      "Use at your own risk.";
      }
    }

    result_times = std::move(std::get<0>(solution));
    next_filter = candidate_times[std::get<1>(solution)].next_node_filter;
    solution_index = std::get<1>(solution);
  }

  return std::make_tuple(next_filter, std::move(result_times), solution_index);
}

void MultiNodeNoncausalOffsetEstimator::CheckInvalidDistance(
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
    return;
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
    FlushAndClose(false);
    LOG(FATAL) << "Please investigate.  Use --max_invalid_distance_ns="
               << invalid_distance.count() << " to ignore this.";
  }
}

std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>
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

    std::vector<BootTimestamp> points(problem->size(),
                                      BootTimestamp::max_time());
    if (VLOG_IS_ON(2)) {
      problem->Debug();
    }
    points[node_a_index] = next_node_time;

    if (!problem->HasObservations(node_a_index)) {
      VLOG(1) << "No observations, checking if there's a filter";
      CHECK(next_node_filter == nullptr)
          << ": No observations, but this isn't a start time.";
      continue;
    }

    std::tuple<std::vector<BootTimestamp>, size_t, size_t> solution =
        problem->SolveNewton(points, kMaxIterations);

    if (std::get<2>(solution) > kMaxIterations) {
      UpdateSolution(std::move(std::get<0>(solution)));
      FlushAndClose(false);
      LOG(FATAL) << "Failed to converge.";
    }

    // Bypass checking if order validation is turned off.  This lets us dump a
    // CSV file so we can view the problem and figure out what to do.  The
    // results won't make sense.
    if (!problem->ValidateSolution(std::get<0>(solution), false)) {
      LOG(WARNING) << "Invalid solution, constraints not met.";
      for (size_t i = 0; i < std::get<0>(solution).size(); ++i) {
        LOG(INFO) << "  " << std::get<0>(solution)[i];
      }
      problem->Debug();
      if (!skip_order_validation_) {
        UpdateSolution(std::get<0>(solution));
        FlushAndClose(false);
        LOG(FATAL) << "Bailing, use --skip_order_validation to continue.  "
                      "Use at your own risk.";
      }
    }

    if (VLOG_IS_ON(1)) {
      VLOG(1) << "Candidate std::get<0>(solution) for node " << node_a_index
              << " is";
      for (size_t i = 0; i < std::get<0>(solution).size(); ++i) {
        VLOG(1) << "  " << std::get<0>(solution)[i];
      }
    }

    if (result_times.empty()) {
      // This is the first solution candidate, so don't bother comparing.
      result_times = std::move(std::get<0>(solution));
      next_filter = next_node_filter;
      solution_index = node_a_index;
      continue;
    }

    switch (CompareTimes(result_times, std::get<0>(solution))) {
      // The old solution is before or at the new solution.  This means that
      // the old solution is a better result, so ignore this one.
      case TimeComparison::kBefore:
      case TimeComparison::kEq:
        break;
      case TimeComparison::kAfter:
        // The new solution is better!  Save it.
        result_times = std::move(std::get<0>(solution));
        next_filter = next_node_filter;
        solution_index = node_a_index;
        break;
      case TimeComparison::kInvalid: {
        CheckInvalidDistance(result_times, std::get<0>(solution));
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

std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>
MultiNodeNoncausalOffsetEstimator::NextSolution(
    TimestampProblem *problem, const std::vector<BootTimestamp> &base_times) {
  // Ok, now solve for the minimum time on each channel.
  std::vector<BootTimestamp> result_times;

  bool boots_all_match = true;
  std::vector<CandidateTimes> candidate_times;
  std::tie(candidate_times, boots_all_match) = MakeCandidateTimes();

  NoncausalTimestampFilter *next_filter = nullptr;
  size_t solution_index = 0;

  // We can significantly speed things up if we know that all the boots match by
  // solving for everything at once.  If the boots don't match, the combined min
  // that happens inside the solver doesn't make a lot of sense since we are
  // actually using the boot from the candidate times to figure out which
  // interpolation function to use under the hood.
  if (boots_all_match) {
    std::tie(next_filter, result_times, solution_index) =
        SimultaneousSolution(problem, std::move(candidate_times), base_times);
  } else {
    // If all the boots don't match, fall back to the old method of comparing
    // all the solutions individually.
    std::tie(next_filter, result_times, solution_index) =
        SequentialSolution(problem, std::move(candidate_times), base_times);
  }
  if (VLOG_IS_ON(1)) {
    VLOG(1) << "Best solution is for node " << solution_index;
    for (size_t i = 0; i < result_times.size(); ++i) {
      VLOG(1) << "  " << result_times[i];
    }
  }
  return std::make_tuple(next_filter, std::move(result_times), solution_index);
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

std::optional<
    std::tuple<distributed_clock::time_point, std::vector<BootTimestamp>>>
MultiNodeNoncausalOffsetEstimator::NextTimestamp() {
  // TODO(austin): Detect and handle there being fewer nodes in the log file
  // than in replay, or them being in a different order.
  TimestampProblem problem = MakeProblem();

  // Ok, now solve for the minimum time on each channel.
  std::vector<BootTimestamp> result_times;
  NoncausalTimestampFilter *next_filter = nullptr;
  int solution_node_index = 0;
  std::tie(next_filter, result_times, solution_node_index) =
      NextSolution(&problem, last_monotonics_);

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
    return std::nullopt;
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
      next_filter->Pop(std::get<0>(sample) - time_estimation_buffer_seconds_);
    }
  } else {
    if (next_filter) {
      // This isn't a start time because we have a corresponding filter.
      sample = *next_filter->Consume();
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
        FlushAndClose(false);
        LOG(FATAL)
            << "Found a solution before the last returned solution on node "
            << solution_node_index;
        break;
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
        FlushAndClose(false);
        LOG(FATAL) << "Please investigate.  Use --max_invalid_distance_ns="
                   << invalid_distance.count() << " to ignore this.";
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
