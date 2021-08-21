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

DEFINE_int32(max_invalid_distance_ns, 0,
             "The max amount of time we will let the solver go backwards.");

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;
using aos::logger::BootDuration;
using aos::logger::BootTimestamp;

const Eigen::IOFormat kHeavyFormat(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                   ", ", ";\n", "[", "]", "[", "]");
}  // namespace

TimestampProblem::TimestampProblem(size_t count) {
  CHECK_GT(count, 1u);
  filters_.resize(count);
  base_clock_.resize(count);
  live_.resize(count, true);
  node_mapping_.resize(count, 0);
}

// TODO(austin): Add linear inequality constraints too.  Currently we just
// enforce them.
//
// TODO(austin): Add a rate of change constraint from the last sample.  1
// ms/s.  Figure out how to define it.  Do this last.  This lets us handle
// constraints going away, and constraints close in time.

bool TimestampProblem::ValidateSolution(std::vector<BootTimestamp> solution) {
  bool success = true;
  for (size_t i = 0u; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      success = success && filter.filter->ValidateSolution(
                               solution[i], solution[filter.b_index]);
    }
  }
  return success;
}

Eigen::VectorXd TimestampProblem::Gradient(
    const Eigen::Ref<Eigen::VectorXd> time_offsets) const {
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(live_nodes_);
  for (size_t i = 0; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      // Reminder, our cost function has the following form.
      //   ((tb - (1 + ma) ta - ba)^2
      // We are ignoring the slope when taking the derivative and applying the
      // chain rule to keep the gradient smooth.  This means that the gradient
      // is +- 2 * error.
      //
      const size_t a_solution_index = NodeToFullSolutionIndex(i);
      const size_t b_solution_index = NodeToFullSolutionIndex(filter.b_index);
      const double error =
          2.0 * filter.filter->OffsetError(base_clock_[i],
                                           time_offsets(a_solution_index),
                                           base_clock_[filter.b_index],
                                           time_offsets(b_solution_index));

      grad(a_solution_index) += -error;
      grad(b_solution_index) += error;
    }
  }
  return grad;
}

Eigen::MatrixXd TimestampProblem::Hessian(
    const Eigen::Ref<Eigen::VectorXd> /*time_offsets*/) const {
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(live_nodes_, live_nodes_);

  for (size_t i = 0; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      // Reminder, our cost function has the following form.
      //   ((tb - (1 + ma) ta - ba)^2
      // We are ignoring the slope when taking the derivative and applying the
      // chain rule to keep the gradient smooth.  This means that the Hessian is
      // 2 for d^2 cost/dta^2 and d^2 cost/dtb^2
      const size_t a_solution_index = NodeToFullSolutionIndex(i);
      const size_t b_solution_index = NodeToFullSolutionIndex(filter.b_index);
      hessian(a_solution_index, a_solution_index) += 2;
      hessian(b_solution_index, a_solution_index) += -2;
      hessian(a_solution_index, b_solution_index) =
          hessian(b_solution_index, a_solution_index);
      hessian(b_solution_index, b_solution_index) += 2;
    }
  }

  return hessian;
}

Eigen::VectorXd TimestampProblem::Newton(
    const Eigen::Ref<Eigen::VectorXd> time_offsets) const {
  CHECK_GT(live_nodes_, 0u) << ": No live nodes to solve for.";
  // TODO(austin): Each of the DCost functions does a binary search of the
  // timestamps list.  By the time we have computed the gradient and Hessian,
  // we've done 5 binary searches for the same information.
  const Eigen::VectorXd grad = Gradient(time_offsets);
  const Eigen::MatrixXd hessian = Hessian(time_offsets);
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
  // TODO(austin): Maybe drive the distributed so we drive the min clock?  This
  // will solve the for loop at the same time, making things faster.
  //
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
  a.block(0, 0, live_nodes_, live_nodes_) = hessian;
  a.block(0, live_nodes_, live_nodes_, 1) = constraint_jacobian.transpose();
  a.block(live_nodes_, 0, 1, live_nodes_) = constraint_jacobian;
  a(live_nodes_, live_nodes_) = 0.0;

  Eigen::VectorXd b = Eigen::VectorXd::Zero(live_nodes_ + 1);
  b.block(0, 0, live_nodes_, 1) = -grad;

  // Since we are driving the clock on the solution node to the base_clock, that
  // is equivalent to driving the solution node's offset to 0.
  b(live_nodes_) = -time_offsets(NodeToFullSolutionIndex(solution_node_));

  return a.colPivHouseholderQr().solve(b);
}

std::vector<BootTimestamp> TimestampProblem::SolveNewton() {
  constexpr int kMaxIterations = 200;
  MaybeUpdateNodeMapping();
  VLOG(1) << "Solving for node " << solution_node_ << " at "
          << base_clock(solution_node_);
  Eigen::VectorXd data = Eigen::VectorXd::Zero(live_nodes_);

  int solution_number = 0;
  while (true) {
    Eigen::VectorXd step = Newton(data);

    if (VLOG_IS_ON(1)) {
      // Print out the gradient ignoring the component removed by the equality
      // constraint.  This tells us what gradient we are depending to try to
      // finish our solution.
      const Eigen::MatrixXd constraint_jacobian =
          Eigen::MatrixXd::Ones(1, live_nodes_) /
          static_cast<double>(live_nodes_);
      Eigen::VectorXd adjusted_grad =
          Gradient(data) + step(live_nodes_) * constraint_jacobian.transpose();

      VLOG(1) << "Adjusted grad " << solution_number << " -> "
              << std::setprecision(12) << std::fixed << std::setfill(' ')
              << adjusted_grad.transpose().format(kHeavyFormat);
    }

    VLOG(1) << "Step " << solution_number << " -> " << std::setprecision(12)
            << std::fixed << std::setfill(' ')
            << step.transpose().format(kHeavyFormat);
    // We got there if the max step is small (this is strongly correlated to the
    // gradient since the Hessian is constant), and our solution node's time is
    // also close.
    if (step.block(0, 0, live_nodes_, 1).lpNorm<Eigen::Infinity>() < 1e-4 &&
        std::abs(data(NodeToFullSolutionIndex(solution_node_))) < 1e-4) {
      break;
    }

    data += step.block(0, 0, live_nodes_, 1);

    ++solution_number;

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
      if (j != solution_node_ && live(j) &&
          std::abs(data(solution_index)) > 1000) {
        int64_t dsolution =
            static_cast<int64_t>(std::round(data(solution_index)));
        base_clock_[j].time += chrono::nanoseconds(dsolution);
        data(solution_index) -= dsolution;
      }
    }

    // And finally, don't let us iterate forever.  If it isn't converging,
    // report back.
    if (solution_number > kMaxIterations) {
      break;
    }
  }

  VLOG(1) << "Solving for node " << solution_node_ << " of "
          << base_clock(solution_node_) << " in " << solution_number
          << " cycles";
  std::vector<BootTimestamp> result(size());
  for (size_t i = 0; i < size(); ++i) {
    if (live(i)) {
      result[i].boot = base_clock(i).boot;
      result[i].time = base_clock(i).time +
                       std::chrono::nanoseconds(static_cast<int64_t>(
                           std::round(data(NodeToFullSolutionIndex(i)))));
      VLOG(1) << "live  " << result[i] << " "
              << data(NodeToFullSolutionIndex(i));
    } else {
      result[i] = BootTimestamp::min_time();
      VLOG(1) << "dead  " << result[i];
    }
  }
  if (solution_number > kMaxIterations) {
    LOG(FATAL) << "Failed to converge.";
  }

  return result;
}

void TimestampProblem::Debug() {
  MaybeUpdateNodeMapping();
  LOG(INFO) << "Solving for node " << solution_node_ << " at "
            << base_clock_[solution_node_];

  std::vector<std::vector<std::string>> gradients(filters_.size());
  for (size_t i = 0u; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      if (live(i) && live(filter.b_index)) {
        // TODO(austin): This should be right, but I haven't gone and spent a
        // bunch of time making sure it all matches perfectly.  We aren't
        // hitting this anymore.  I'm also likely the one who will be debugging
        // it next and would rather spend the time debugging it when I get a bug
        // report.
        gradients[i].emplace_back(
            std::string("- ") +
            filter.filter->DebugOffsetError(base_clock_[i], 0.0,
                                            base_clock_[filter.b_index], 0.0, i,
                                            filter.b_index));
        gradients[filter.b_index].emplace_back(filter.filter->DebugOffsetError(
            base_clock_[i], 0.0, base_clock_[filter.b_index], 0.0, i,
            filter.b_index));
      }
    }
  }

  for (size_t i = 0u; i < filters_.size(); ++i) {
    LOG(INFO) << (live(i) ? "live" : "dead") << " Grad[" << i << "] = "
              << (gradients[i].empty() ? std::string("0.0")
                                       : absl::StrJoin(gradients[i], " + "));
  }

  for (size_t i = 0u; i < filters_.size(); ++i) {
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

  // TODO(austin): Figure out how to communicate the reboot up to the factory.
  CHECK_EQ(node_count_, std::get<1>(*next_time).size());
  times_.emplace_back(std::move(*next_time));
  return &times_.back();
}

void InterpolatedTimeConverter::QueueUntil(
    std::function<bool(const std::tuple<distributed_clock::time_point,
                                        std::vector<BootTimestamp>> &)>
        not_done) {
  while (!at_end_ && (times_.empty() || not_done(times_.back()))) {
    QueueNextTimestamp();
  }

  CHECK(!times_.empty())
      << ": Found no times to do timestamp estimation, please investigate.";
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
    size_t node_index, monotonic_clock::time_point time) {
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
        return std::get<1>(t)[node_index].time < time;
      });

  // Before the beginning needs to have 0 slope otherwise time jumps when
  // timestamp 2 happens.
  if (times_.size() == 1u || time < std::get<1>(times_[0])[node_index].time) {
    if (time < std::get<1>(times_[0])[node_index].time) {
      CHECK(!have_popped_)
          << ": Trying to interpolate time " << time
          << " but we have forgotten the relevant points already.";
    }
    const distributed_clock::time_point result =
        time - std::get<1>(times_[0])[node_index].time + std::get<0>(times_[0]);
    VLOG(2) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
            << result;
    return result;
  }

  // Now, find the corresponding timestamps.  Search from the back since that's
  // where most of the times we care about will be.
  size_t index = times_.size() - 2u;
  while (index > 0u) {
    // TODO(austin): Binary search.
    if (std::get<1>(times_[index])[node_index].time <= time) {
      break;
    }
    --index;
  }

  // Interpolate with the two of these.
  const distributed_clock::time_point d0 = std::get<0>(times_[index]);
  const distributed_clock::time_point d1 = std::get<0>(times_[index + 1]);

  // TODO(austin): We should extrapolate if the boot changes.
  CHECK_EQ(std::get<1>(times_[index])[node_index].boot,
           std::get<1>(times_[index + 1])[node_index].boot);
  const monotonic_clock::time_point t0 =
      std::get<1>(times_[index])[node_index].time;
  const monotonic_clock::time_point t1 =
      std::get<1>(times_[index + 1])[node_index].time;

  const distributed_clock::time_point result =
      message_bridge::ToDistributedClock(d0, d1, t0, t1, time);

  VLOG(2) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
          << result;
  return result;
}

monotonic_clock::time_point InterpolatedTimeConverter::FromDistributedClock(
    size_t node_index, distributed_clock::time_point time) {
  CHECK_LT(node_index, node_count_);
  // If there is only one node, time estimation makes no sense.  Just return
  // unity time.
  if (node_count_ == 1u) {
    return monotonic_clock::epoch() + time.time_since_epoch();
  }

  // Make sure there are enough timestamps in the queue.
  QueueUntil(
      [time](const std::tuple<distributed_clock::time_point,
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
    VLOG(2) << "FromDistributedClock(" << node_index << ", " << time << ") -> "
            << result;
    return result;
  }

  // Now, find the corresponding timestamps.  Search from the back since that's
  // where most of the times we care about will be.
  size_t index = times_.size() - 2u;
  while (index > 0u) {
    if (std::get<0>(times_[index]) <= time) {
      break;
    }
    --index;
  }

  // Interpolate with the two of these.
  const distributed_clock::time_point d0 = std::get<0>(times_[index]);
  const distributed_clock::time_point d1 = std::get<0>(times_[index + 1]);

  CHECK_EQ(std::get<1>(times_[index])[node_index].boot,
           std::get<1>(times_[index + 1])[node_index].boot);
  const monotonic_clock::time_point t0 =
      std::get<1>(times_[index])[node_index].time;
  const monotonic_clock::time_point t1 =
      std::get<1>(times_[index + 1])[node_index].time;

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
  absl::int128 numerator =
      absl::int128((time - d0).count()) * absl::int128((t1 - t0).count());
  numerator += numerator > 0 ? absl::int128(dd.count() / 2)
                             : -absl::int128(dd.count() / 2);

  const monotonic_clock::time_point result =
      t0 + std::chrono::nanoseconds(
               static_cast<int64_t>(numerator / absl::int128(dd.count())));
  VLOG(2) << "FromDistributedClock(" << node_index << ", " << time << ") -> "
          << result;
  return result;
}
MultiNodeNoncausalOffsetEstimator::MultiNodeNoncausalOffsetEstimator(
    const Configuration *configuration,
    const Configuration *logged_configuration, bool skip_order_validation,
    chrono::nanoseconds time_estimation_buffer_seconds)
    : InterpolatedTimeConverter(!configuration::MultiNode(logged_configuration)
                                    ? 1u
                                    : logged_configuration->nodes()->size(),
                                time_estimation_buffer_seconds),
      configuration_(configuration),
      logged_configuration_(logged_configuration),
      skip_order_validation_(skip_order_validation) {
  filters_per_node_.resize(NodesCount());
  last_monotonics_.resize(NodesCount(), BootTimestamp::epoch());
  if (FLAGS_timestamps_to_csv &&
      configuration::MultiNode(logged_configuration)) {
    fp_ = fopen("/tmp/timestamp_noncausal_offsets.csv", "w");
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

MultiNodeNoncausalOffsetEstimator::~MultiNodeNoncausalOffsetEstimator() {
  FlushAllSamples(true);
  if (fp_) {
    fclose(fp_);
    fp_ = NULL;
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
        CHECK (timestamps.messages.empty());
      }
    }
  }
}

void MultiNodeNoncausalOffsetEstimator::Start(
    SimulatedEventLoopFactory *factory) {
  std::vector<monotonic_clock::time_point> times;
  for (const Node *node : configuration::GetNodes(factory->configuration())) {
    times.emplace_back(factory->GetNodeEventLoopFactory(node)->monotonic_now());
  }
  Start(times);
}

void MultiNodeNoncausalOffsetEstimator::Start(
    std::vector<monotonic_clock::time_point> times) {
  std::fstream s("/tmp/timestamp_noncausal_starttime.csv", s.trunc | s.out);
  CHECK(s.is_open());
  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    s << node->name()->string_view() << ", " << std::setprecision(12)
      << std::fixed
      << chrono::duration<double>(times[node_index].time_since_epoch()).count()
      << "\n";
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
    filters_per_node_[node_a_index].emplace_back(x.GetFilter(node_a),
                                                 node_b_index);
    filters_per_node_[node_b_index].emplace_back(x.GetFilter(node_b),
                                                 node_a_index);
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
          problem.add_filter(node_a_index, filter.filter, filter.b_index);

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

std::tuple<NoncausalTimestampFilter *, std::vector<BootTimestamp>, int>
MultiNodeNoncausalOffsetEstimator::NextSolution(
    TimestampProblem *problem, const std::vector<BootTimestamp> &base_times) {
  // Ok, now solve for the minimum time on each channel.
  std::vector<BootTimestamp> result_times;
  NoncausalTimestampFilter *next_filter = nullptr;
  size_t solution_index = 0;
  {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      VLOG(1) << "Investigating filter for node " << node_a_index;
      BootTimestamp next_node_time = BootTimestamp::max_time();
      BootDuration next_node_duration;
      NoncausalTimestampFilter *next_node_filter = nullptr;
      // Find the oldest time for each node in each filter, and solve for that
      // time.  That gives us the next timestamp for this node.
      size_t filter_index = 0;
      for (const auto &filter : filters) {
        std::optional<std::tuple<BootTimestamp, BootDuration>> candidate =
            filter.filter->Observe();

        if (candidate) {
          VLOG(1) << "Candidate for node " << node_a_index << " filter "
                  << filter_index << " is " << std::get<0>(*candidate);
          if (std::get<0>(*candidate) < next_node_time) {
            next_node_time = std::get<0>(*candidate);
            next_node_duration = std::get<1>(*candidate);
            next_node_filter = filter.filter;
          }
        }
        ++filter_index;
      }

      // Found no active filters.  Either this node is off, or disconnected, or
      // we are before the log file starts or after the log file ends.
      if (next_node_time == BootTimestamp::max_time()) {
        ++node_a_index;
        continue;
      }
      VLOG(1) << "Trying " << next_node_time << " " << next_node_duration
              << " for node " << node_a_index;

      // TODO(austin): If we start supporting only having 1 direction of
      // timestamps, we might need to change our assumptions around
      // BootTimestamp and BootDuration.

      // If we haven't rebooted, we can seed the optimization problem with a
      // pretty good initial guess.
      if (next_node_time.boot == base_times[node_a_index].boot) {
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
      }

      problem->set_solution_node(node_a_index);
      problem->set_base_clock(problem->solution_node(), next_node_time);
      if (VLOG_IS_ON(2)) {
        problem->Debug();
      }
      // TODO(austin): Solve all problems at once :)
      std::vector<BootTimestamp> solution = problem->SolveNewton();

      // Bypass checking if order validation is turned off.  This lets us dump a
      // CSV file so we can view the problem and figure out what to do.  The
      // results won't make sense.
      if (!problem->ValidateSolution(solution)) {
        LOG(WARNING) << "Invalid solution, constraints not met.";
        for (size_t i = 0; i < solution.size(); ++i) {
          LOG(INFO) << "  " << solution[i];
        }
        problem->Debug();
        if (!skip_order_validation_) {
          LOG(FATAL) << "Bailing, use --skip_order_validation to continue.  "
                        "Use at your own risk.";
        }
      }

      if (VLOG_IS_ON(1)) {
        VLOG(1) << "Candidate solution for node " << node_a_index << " is";
        for (size_t i = 0; i < solution.size(); ++i) {
          VLOG(1) << "  " << solution[i];
        }
      }
      if (result_times.empty()) {
        result_times = std::move(solution);
        next_filter = next_node_filter;
        solution_index = node_a_index;
        ++node_a_index;
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
          // If times are close enough, drop the invalid time.
          const chrono::nanoseconds invalid_distance =
              InvalidDistance(result_times, solution);
          if (invalid_distance <=
              chrono::nanoseconds(FLAGS_max_invalid_distance_ns)) {
            VLOG(1) << "Times can't be compared by " << invalid_distance.count()
                    << "ns";
            for (size_t i = 0; i < result_times.size(); ++i) {
              VLOG(1) << "  " << result_times[i] << " vs " << solution[i]
                      << " -> "
                      << (result_times[i].time - solution[i].time).count()
                      << "ns";
            }
            VLOG(1) << "Ignoring because it is close enough.";
            next_node_filter->Consume();
            break;
          }
          // Somehow the new solution is better *and* worse than the old
          // solution...  This is an internal failure because that means time
          // goes backwards on a node.
          CHECK_EQ(result_times.size(), solution.size());
          LOG(INFO) << "Times can't be compared by " << invalid_distance.count()
                    << "ns";
          for (size_t i = 0; i < result_times.size(); ++i) {
            LOG(INFO) << "  " << result_times[i] << " vs " << solution[i]
                      << " -> "
                      << (result_times[i].time - solution[i].time).count()
                      << "ns";
          }

          if (skip_order_validation_) {
            next_node_filter->Consume();
            LOG(ERROR) << "Skipping because --skip_order_validation";
            break;
          } else {
            LOG(FATAL) << "Please investigate.  Use --max_invalid_distance_ns="
                       << invalid_distance.count() << " to ignore this.";
          }
        } break;
      }
      ++node_a_index;
    }
  }
  if (VLOG_IS_ON(1)) {
    VLOG(1) << "Best solution is for node " << solution_index;
    for (size_t i = 0; i < result_times.size(); ++i) {
      VLOG(1) << "  " << result_times[i];
    }
  }
  return std::make_tuple(next_filter, std::move(result_times), solution_index);
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
  if (next_filter == nullptr) {
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
    sample = *next_filter->Consume();
  } else {
    sample = *next_filter->Consume();
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
        LOG(FATAL)
            << "Found a solution before the last returned solution on node "
            << solution_node_index;
        break;
      case TimeComparison::kEq:
        return NextTimestamp();
      case TimeComparison::kInvalid: {
        const chrono::nanoseconds invalid_distance =
            InvalidDistance(last_monotonics_, result_times);
        if (invalid_distance <
            chrono::nanoseconds(FLAGS_max_invalid_distance_ns)) {
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
        LOG(FATAL) << "Please investigate.  Use --max_invalid_distance_ns="
                   << invalid_distance.count() << " to ignore this.";
      } break;
    }
  }

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

  if (filter_fps_.size() > 0) {
    const int node_a_index =
        configuration::GetNodeIndex(configuration(), next_filter->node_a());
    const int node_b_index =
        configuration::GetNodeIndex(configuration(), next_filter->node_b());

    FILE *fp = filter_fps_[node_a_index][node_b_index];
    if (fp == nullptr) {
      fp = filter_fps_[node_a_index][node_b_index] = fopen(
          absl::StrCat("/tmp/timestamp_noncausal_",
                       next_filter->node_a()->name()->string_view(), "_",
                       next_filter->node_b()->name()->string_view(), ".csv")
              .c_str(),
          "w");
      fprintf(fp, "time_since_start,sample_ns,filtered_offset\n");
    }

    fprintf(fp, "%.9f, %.9f, %.9f\n",
            std::chrono::duration_cast<std::chrono::duration<double>>(
                last_distributed_.time_since_epoch())
                .count(),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::get<0>(sample).time.time_since_epoch())
                .count(),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::get<1>(sample).duration)
                .count());
  }

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
            fopen(absl::StrCat("/tmp/timestamp_noncausal_",
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
