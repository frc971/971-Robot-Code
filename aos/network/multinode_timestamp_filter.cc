#include "aos/network/multinode_timestamp_filter.h"

#include <chrono>
#include <functional>
#include <map>

#include "absl/strings/str_join.h"
#include "aos/configuration.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "glog/logging.h"
#include "nlopt.h"

DEFINE_bool(timestamps_to_csv, false,
            "If true, write all the time synchronization information to a set "
            "of CSV files in /tmp/.  This should only be needed when debugging "
            "time synchronization.");

DEFINE_int32(max_invalid_distance_ns, 500,
             "The max amount of time we will let the solver go backwards.");

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;
}

TimestampProblem::TimestampProblem(size_t count) {
  CHECK_GT(count, 1u);
  filters_.resize(count);
  base_clock_.resize(count);
  live_.resize(count, true);
  node_mapping_.resize(count, 0);
}

// TODO(austin): Add linear inequality constraints too.
//
// TODO(austin): Add a rate of change constraint from the last sample.  1
// ms/s.  Figure out how to define it.  Do this last.  This lets us handle
// constraints going away, and constraints close in time.
//
// TODO(austin): Use the timestamp of the remote timestamp as more data.

std::vector<double> TimestampProblem::SolveDouble() {
  MaybeUpdateNodeMapping();
  // TODO(austin): Add constraints for relevant segments.
  const size_t n = filters_.size() - 1u;
  //  NLOPT_LD_MMA and NLOPT_LD_LBFGS are alternative solvers, but SLSQP is a
  //  better fit for the quadratic nature of this problem.
  nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, n);
  nlopt_set_min_objective(opt, TimestampProblem::DoCost, this);

  // Ask for really good.  This is very quadratic, so it should be pretty
  // precise.
  nlopt_set_xtol_rel(opt, 1e-9);

  cost_call_count_ = 0;

  std::vector<double> result(n, 0.0);
  double minf = 0.0;
  nlopt_result status = nlopt_optimize(opt, result.data(), &minf);
  if (status < 0) {
    if (status == NLOPT_ROUNDOFF_LIMITED) {
      constexpr double kTolerance = 1e-9;
      std::vector<double> gradient(n, 0.0);
      Cost(result.data(), gradient.data());
      for (double g : gradient) {
        if (std::abs(g) > kTolerance) {
          // If we failed, update base_clock_ to the current time so it gets
          // printed inside Debug and explode with a CHECK saying the same
          // thing.
          std::vector<monotonic_clock::time_point> new_base =
              DoubleToMonotonic(result.data());
          // Put the result into base_clock_ so Debug prints out something
          // useful.
          base_clock_ = std::move(new_base);
          Debug();
          CHECK_LE(std::abs(g), kTolerance)
              << ": Optimization problem failed with a large gradient.  "
              << nlopt_result_to_string(status);
        }
      }
    } else {
      LOG(FATAL) << "Failed to solve optimization problem "
                 << nlopt_result_to_string(status);
    }
  }

  if (VLOG_IS_ON(1)) {
    PrintSolution(result);
  }
  nlopt_destroy(opt);
  return result;
}

void TimestampProblem::PrintSolution(const std::vector<double> &solution) {
  const size_t n = filters_.size() - 1u;
  std::vector<double> gradient(n, 0.0);
  const double minf = Cost(solution.data(), gradient.data());

  // High precision formatter for the gradient.
  struct MyFormatter {
    void operator()(std::string *out, double i) const {
      std::stringstream ss;
      ss << std::setprecision(12) << std::fixed << i;
      out->append(ss.str());
    }
  };

  LOG(INFO) << std::setprecision(12) << std::fixed << "Found minimum at f("
            << absl::StrJoin(solution, ", ") << ") -> " << minf << " grad ["
            << absl::StrJoin(gradient, ", ", MyFormatter()) << "] after "
            << cost_call_count_ << " cycles for node " << solution_node_ << ".";
}

std::vector<monotonic_clock::time_point> TimestampProblem::DoubleToMonotonic(
    const double *r) const {
  std::vector<monotonic_clock::time_point> result(filters_.size());
  for (size_t i = 0; i < result.size(); ++i) {
    if (live(i)) {
      result[i] = base_clock(i) + std::chrono::nanoseconds(static_cast<int64_t>(
                                      std::round(get_t(r, i))));
    } else {
      result[i] = monotonic_clock::min_time;
    }
  }

  return result;
}

std::vector<monotonic_clock::time_point> TimestampProblem::Solve() {
  std::vector<double> solution = SolveDouble();
  return DoubleToMonotonic(solution.data());
}

bool TimestampProblem::ValidateSolution(
    std::vector<monotonic_clock::time_point> solution) {
  bool success = true;
  for (size_t i = 0u; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      success = success && filter.filter->ValidateSolution(
                               solution[i], solution[filter.b_index]);
    }
  }
  return success;
}

double TimestampProblem::Cost(const double *time_offsets, double *grad) {
  ++cost_call_count_;

  if (grad != nullptr) {
    for (size_t i = 0; i < filters_.size() - 1u; ++i) {
      grad[i] = 0;
    }

    for (size_t i = 0u; i < filters_.size(); ++i) {
      for (const struct FilterPair &filter : filters_[i]) {
        if (i != solution_node_) {
          grad[NodeToSolutionIndex(i)] += filter.filter->DCostDta(
              base_clock_[i], get_t(time_offsets, i),
              base_clock_[filter.b_index], get_t(time_offsets, filter.b_index));
        }
        if (filter.b_index != solution_node_) {
          grad[NodeToSolutionIndex(filter.b_index)] += filter.filter->DCostDtb(
              base_clock_[i], get_t(time_offsets, i),
              base_clock_[filter.b_index], get_t(time_offsets, filter.b_index));
        }
      }
    }
  }

  double cost = 0;
  for (size_t i = 0u; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      cost += filter.filter->Cost(base_clock_[i], get_t(time_offsets, i),
                                  base_clock_[filter.b_index],
                                  get_t(time_offsets, filter.b_index));
    }
  }

  if (VLOG_IS_ON(1)) {
    struct MyFormatter {
      void operator()(std::string *out, monotonic_clock::time_point t) const {
        std::stringstream ss;
        ss << t;
        out->append(ss.str());
      }
      void operator()(std::string *out, double i) const {
        std::stringstream ss;
        ss << std::setprecision(12) << std::fixed << i;
        out->append(ss.str());
      }
    };

    std::string gradient;
    if (grad) {
      std::stringstream ss;
      ss << " grad ["
         << absl::StrJoin(absl::Span<const double>(grad, LiveNodesCount() - 1u),
                          ", ", MyFormatter())
         << "]";
      gradient = ss.str();
    }

    LOG(INFO) << std::setprecision(12) << std::fixed
              << "Evaluated minimum at f("
              << absl::StrJoin(DoubleToMonotonic(time_offsets), ", ",
                               MyFormatter())
              << ") -> " << cost << gradient << " after " << cost_call_count_
              << " cycles.";
  }
  return cost;
}

void TimestampProblem::Debug() {
  MaybeUpdateNodeMapping();
  LOG(INFO) << "Solving for node " << solution_node_ << " at "
            << base_clock_[solution_node_];

  std::vector<std::string> cost;
  for (size_t i = 0u; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      cost.emplace_back(filter.filter->DebugCost(base_clock_[i], 0.0,
                                                 base_clock_[filter.b_index],
                                                 0.0, i, filter.b_index));
    }
  }
  LOG(INFO) << "Cost: " << absl::StrJoin(cost, " + ");

  std::vector<std::vector<std::string>> gradients(filters_.size());
  for (size_t i = 0u; i < filters_.size(); ++i) {
    std::string gradient = "0.0";
    for (const struct FilterPair &filter : filters_[i]) {
      if (i != solution_node_ && live(i)) {
        gradients[i].emplace_back(filter.filter->DebugDCostDta(
            base_clock_[i], 0.0, base_clock_[filter.b_index], 0.0, i,
            filter.b_index));
      }
      if (filter.b_index != solution_node_ && live(filter.b_index)) {
        gradients[filter.b_index].emplace_back(filter.filter->DebugDCostDtb(
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

void InterpolatedTimeConverter::QueueUntil(
    std::function<
        bool(const std::tuple<distributed_clock::time_point,
                              std::vector<monotonic_clock::time_point>> &)>
        not_done) {
  while (!at_end_ && (times_.empty() || not_done(times_.back()))) {
    std::optional<std::tuple<distributed_clock::time_point,
                             std::vector<monotonic_clock::time_point>>>
        next_time = NextTimestamp();
    if (!next_time) {
      LOG(INFO) << "Last timestamp, calling it quits";
      at_end_ = true;
      break;
    }
    VLOG(1) << "Fetched next timestamp while solving: "
            << std::get<0>(*next_time) << " ->";
    for (monotonic_clock::time_point t : std::get<1>(*next_time)) {
      VLOG(1) << "  " << t;
    }
    CHECK_EQ(node_count_, std::get<1>(*next_time).size());
    times_.emplace_back(std::move(*next_time));
  }

  CHECK(!times_.empty())
      << ": Found no times to do timestamp estimation, please investigate.";
  // Keep at least 500 points and time_estimation_buffer_seconds seconds of
  // time.  This should be enough to handle any reasonable amount of history.
  while (times_.size() > kHistoryMinCount &&
         std::get<0>(times_.front()) + time_estimation_buffer_seconds_ <
             std::get<0>(times_.back())) {
    times_.pop_front();
    have_popped_ = true;
  }
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
      [time, node_index](
          const std::tuple<distributed_clock::time_point,
                           std::vector<monotonic_clock::time_point>> &t) {
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
    const distributed_clock::time_point result =
        time - std::get<1>(times_[0])[node_index] + std::get<0>(times_[0]);
    VLOG(2) << "ToDistributedClock(" << node_index << ", " << time << ") -> "
            << result;
    return result;
  }

  // Now, find the corresponding timestamps.  Search from the back since that's
  // where most of the times we care about will be.
  size_t index = times_.size() - 2u;
  while (index > 0u) {
    if (std::get<1>(times_[index])[node_index] <= time) {
      break;
    }
    --index;
  }

  // Interpolate with the two of these.
  const distributed_clock::time_point d0 = std::get<0>(times_[index]);
  const distributed_clock::time_point d1 = std::get<0>(times_[index + 1]);

  const monotonic_clock::time_point t0 = std::get<1>(times_[index])[node_index];
  const monotonic_clock::time_point t1 =
      std::get<1>(times_[index + 1])[node_index];

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
  const distributed_clock::time_point result =
      d0 + std::chrono::nanoseconds(
               static_cast<int64_t>(numerator / absl::int128(dt.count())));
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
                              std::vector<monotonic_clock::time_point>> &t) {
        return std::get<0>(t) < time;
      });

  if (times_.size() == 1u || time < std::get<0>(times_[0])) {
    if (time < std::get<0>(times_[0])) {
      CHECK(!have_popped_)
          << ": Trying to interpolate time " << time
          << " but we have forgotten the relevant points already.";
    }
    monotonic_clock::time_point result =
        time - std::get<0>(times_[0]) + std::get<1>(times_[0])[node_index];
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

  const monotonic_clock::time_point t0 = std::get<1>(times_[index])[node_index];
  const monotonic_clock::time_point t1 =
      std::get<1>(times_[index + 1])[node_index];

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
  last_monotonics_.resize(NodesCount(), aos::monotonic_clock::epoch());
  if (FLAGS_timestamps_to_csv &&
      configuration::MultiNode(logged_configuration)) {
    fp_ = fopen("/tmp/timestamp_noncausal_offsets.csv", "w");
    fprintf(fp_, "# distributed");
    for (const Node *node : configuration::GetNodes(logged_configuration)) {
      fprintf(fp_, ", %s", node->name()->c_str());
    }
    fprintf(fp_, "\n");
  }
}

MultiNodeNoncausalOffsetEstimator::~MultiNodeNoncausalOffsetEstimator() {
  if (fp_) {
    fclose(fp_);
    fp_ = NULL;
  }
  if (all_done_) {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      for (const auto &filter : filters) {
        std::optional<
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
            next = filter.filter->Consume();
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
  for (std::pair<const std::tuple<const Node *, const Node *>,
                 message_bridge::NoncausalOffsetEstimator> &filter : filters_) {
    const Node *const node_a = std::get<0>(filter.first);
    const size_t node_a_index =
        configuration::GetNodeIndex(configuration_, node_a);
    const Node *const node_b = std::get<1>(filter.first);
    const size_t node_b_index =
        configuration::GetNodeIndex(configuration_, node_b);

    filter.second.SetFirstFwdTime(times[node_a_index]);
    filter.second.SetFirstRevTime(times[node_b_index]);
  }

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
                  .insert(std::make_pair(
                      tuple,
                      message_bridge::NoncausalOffsetEstimator(node_a, node_b)))
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

    if (FLAGS_timestamps_to_csv) {
      x.SetFwdCsvFileName(absl::StrCat("/tmp/timestamp_noncausal_",
                                       node_a->name()->string_view(), "_",
                                       node_b->name()->string_view()));
      x.SetRevCsvFileName(absl::StrCat("/tmp/timestamp_noncausal_",
                                       node_b->name()->string_view(), "_",
                                       node_a->name()->string_view()));
    }

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
      CHECK_EQ(timestamp_mapper->sorted_until(), monotonic_clock::min_time)
          << ": Timestamps queued before we registered the timestamp hooks.";
      timestamp_mapper->set_timestamp_callback(
          [this, node_index](logger::TimestampedMessage *msg) {
            if (msg->monotonic_remote_time != monotonic_clock::min_time) {
              // Got a forwarding timestamp!
              NoncausalOffsetEstimator *filter =
                  filters_per_channel_[node_index][msg->channel_index];
              CHECK_NOTNULL(filter);
              const Node *node = configuration()->nodes()->Get(node_index);

              // Call the correct method depending on if we are the forward or
              // reverse direction here.
              filter->Sample(node, msg->monotonic_event_time,
                             msg->monotonic_remote_time);

              if (msg->monotonic_timestamp_time != monotonic_clock::min_time) {
                // TODO(austin): This assumes that this timestamp is only logged
                // on the node which sent the data.  That is correct for now,
                // but should be explicitly checked somewhere.
                filter->ReverseSample(node, msg->monotonic_event_time,
                                      msg->monotonic_timestamp_time);
              }
            }
          });
    }
    ++node_index;
  }

  timestamp_mappers_ = std::move(timestamp_mappers);
}

TimeComparison CompareTimes(
    const std::vector<monotonic_clock::time_point> &ta,
    const std::vector<monotonic_clock::time_point> &tb) {
  if (ta.size() != tb.size() || ta.empty()) {
    return TimeComparison::kInvalid;
  }
  bool is_less = false;
  bool is_greater = false;
  bool is_eq = true;
  bool some_eq = false;
  for (size_t i = 0; i < ta.size(); ++i) {
    if (tb[i] == monotonic_clock::min_time ||
        ta[i] == monotonic_clock::min_time) {
      continue;
    }
    if (ta[i] < tb[i]) {
      is_less = true;
      is_eq = false;
    } else if (ta[i] > tb[i]) {
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

chrono::nanoseconds MaxElapsedTime(
    const std::vector<monotonic_clock::time_point> &ta,
    const std::vector<monotonic_clock::time_point> &tb) {
  CHECK_EQ(ta.size(), tb.size());
  CHECK(!ta.empty());
  bool first = true;
  chrono::nanoseconds dt;
  for (size_t i = 0; i < ta.size(); ++i) {
    // Skip any invalid timestamps.
    if (ta[i] == monotonic_clock::min_time ||
        tb[i] == monotonic_clock::min_time) {
      continue;
    }

    const chrono::nanoseconds dti = tb[i] - ta[i];
    if (first || dti > dt) {
      dt = dti;
    }
    first = false;
  }
  return dt;
}

chrono::nanoseconds InvalidDistance(
    const std::vector<monotonic_clock::time_point> &ta,
    const std::vector<monotonic_clock::time_point> &tb) {
  // Use an int128 so we have no concern about number of times or size of the
  // difference.
  absl::int128 sum = 0;
  for (size_t i = 0; i < ta.size(); ++i) {
    if (ta[i] == monotonic_clock::min_time ||
        tb[i] == monotonic_clock::min_time) {
      continue;
    }
    sum += (ta[i] - tb[i]).count();
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

std::tuple<NoncausalTimestampFilter *,
           std::vector<aos::monotonic_clock::time_point>>
MultiNodeNoncausalOffsetEstimator::NextSolution(
    TimestampProblem *problem,
    const std::vector<aos::monotonic_clock::time_point> &base_times) {
  // Ok, now solve for the minimum time on each channel.
  std::vector<aos::monotonic_clock::time_point> result_times;
  NoncausalTimestampFilter *next_filter = nullptr;
  size_t solution_index = 0;
  {
    size_t node_a_index = 0;
    for (const auto &filters : filters_per_node_) {
      VLOG(1) << "Investigating filter for node " << node_a_index;
      monotonic_clock::time_point next_node_time = monotonic_clock::max_time;
      NoncausalTimestampFilter *next_node_filter = nullptr;
      // Find the oldest time for each node in each filter, and solve for that
      // time.  That gives us the next timestamp for this node.
      for (const auto &filter : filters) {
        std::optional<
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
            candidate = filter.filter->Observe();

        if (candidate) {
          if (std::get<0>(*candidate) < next_node_time) {
            next_node_time = std::get<0>(*candidate);
            next_node_filter = filter.filter;
          }
        }
      }

      // Found no active filters.  Either this node is off, or disconnected, or
      // we are before the log file starts or after the log file ends.
      if (next_node_time == monotonic_clock::max_time) {
        ++node_a_index;
        continue;
      }

      // Optimize, and save the time into times if earlier than time.
      for (size_t node_index = 0; node_index < base_times.size();
           ++node_index) {
        // Offset everything based on the elapsed time since the last solution
        // on the node we are solving for.  The rate that time elapses should be
        // ~1.
        problem->set_base_clock(
            node_index, base_times[node_index] +
                            (next_node_time - base_times[node_a_index]));
      }

      problem->set_solution_node(node_a_index);
      problem->set_base_clock(problem->solution_node(), next_node_time);
      if (VLOG_IS_ON(1)) {
        problem->Debug();
      }
      // TODO(austin): Can we cache?  Solving is expensive.
      std::vector<monotonic_clock::time_point> solution = problem->Solve();

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
          if (InvalidDistance(result_times, solution) <
              chrono::nanoseconds(FLAGS_max_invalid_distance_ns)) {
            VLOG(1) << "Times can't be compared by "
                    << InvalidDistance(result_times, solution).count() << "ns";
            for (size_t i = 0; i < result_times.size(); ++i) {
              VLOG(1) << "  " << result_times[i] << " vs " << solution[i]
                      << " -> " << (result_times[i] - solution[i]).count()
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
          LOG(INFO) << "Times can't be compared by "
                    << InvalidDistance(result_times, solution).count() << "ns";
          for (size_t i = 0; i < result_times.size(); ++i) {
            LOG(INFO) << "  " << result_times[i] << " vs " << solution[i]
                      << " -> " << (result_times[i] - solution[i]).count()
                      << "ns";
          }
          // Since we found a problem with the solution, solve one problem per
          // node, starting at the problem point.  This will show us any
          // inconsistencies due to the problem phrasing and which node we
          // solved from.
          for (size_t a_index = 0; a_index < solution.size(); ++a_index) {
            if (!problem->live(a_index)) {
              continue;
            }
            for (size_t node_index = 0; node_index < solution.size();
                 ++node_index) {
              // Offset everything based on the elapsed time since the last
              // solution on the node we are solving for.  The rate that time
              // elapses should be ~1.
              problem->set_base_clock(node_index, solution[node_index]);
            }

            problem->set_solution_node(a_index);
            problem->Debug();
            const std::vector<double> resolve_solution_double =
                problem->SolveDouble();
            problem->PrintSolution(resolve_solution_double);

            const std::vector<monotonic_clock::time_point> resolve_solution =
                problem->DoubleToMonotonic(resolve_solution_double.data());

            LOG(INFO) << "Candidate solution for resolved node " << a_index
                      << " is";
            for (size_t i = 0; i < resolve_solution.size(); ++i) {
              LOG(INFO) << "  " << resolve_solution[i] << " vs original "
                        << solution[i] << " -> "
                        << (resolve_solution[i] - solution[i]).count();
            }
          }

          if (skip_order_validation_) {
            next_node_filter->Consume();
            LOG(ERROR) << "Skipping because --skip_order_validation";
            break;
          } else {
            LOG(FATAL) << "Please investigate.  Use --max_invalid_distance_ns "
                          "to change this.";
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
  return std::make_tuple(next_filter, std::move(result_times));
}

std::optional<std::tuple<distributed_clock::time_point,
                         std::vector<monotonic_clock::time_point>>>
MultiNodeNoncausalOffsetEstimator::NextTimestamp() {
  // TODO(austin): Detect and handle there being fewer nodes in the log file
  // than in replay, or them being in a different order.
  TimestampProblem problem = MakeProblem();

  // Ok, now solve for the minimum time on each channel.
  std::vector<aos::monotonic_clock::time_point> result_times;
  NoncausalTimestampFilter *next_filter = nullptr;
  std::tie(next_filter, result_times) =
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
      return std::make_tuple(distributed_clock::epoch(),
                             std::vector<monotonic_clock::time_point>(
                                 NodesCount(), monotonic_clock::epoch()));
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

  if (first_solution_) {
    std::vector<aos::monotonic_clock::time_point> resolved_times;
    NoncausalTimestampFilter *resolved_next_filter = nullptr;

    VLOG(1) << "Resolving with updated base times for accuracy.";
    std::tie(resolved_next_filter, resolved_times) =
        NextSolution(&problem, result_times);

    first_solution_ = false;
    next_filter = resolved_next_filter;

    // Force any unknown nodes to track the distributed clock (which starts at 0
    // too).
    for (monotonic_clock::time_point &time : result_times) {
      if (time == monotonic_clock::min_time) {
        time = monotonic_clock::epoch();
      }
    }
    result_times = std::move(resolved_times);
    next_filter->Consume();
  } else {
    next_filter->Consume();
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
          LOG(INFO) << "  " << last_monotonics_[i] << " vs " << result_times[i];
        }
        LOG(FATAL) << "Found a solution before the last returned solution.";
        break;
      case TimeComparison::kEq:
        return NextTimestamp();
      case TimeComparison::kInvalid:
        if (InvalidDistance(last_monotonics_, result_times) <
            chrono::nanoseconds(FLAGS_max_invalid_distance_ns)) {
          return NextTimestamp();
        }
        CHECK_EQ(last_monotonics_.size(), result_times.size());
        for (size_t i = 0; i < result_times.size(); ++i) {
          LOG(INFO) << "  " << last_monotonics_[i] << " vs " << result_times[i];
        }
        LOG(FATAL) << "Found solutions which can't be ordered.";
        break;
    }
  }

  // Now, figure out what distributed should be.  It should move at the rate of
  // the max elapsed time so that conversions to and from it don't round to bad
  // values.
  const chrono::nanoseconds dt = MaxElapsedTime(last_monotonics_, result_times);
  last_distributed_ += dt;
  for (size_t i = 0; i < result_times.size(); ++i) {
    if (result_times[i] == monotonic_clock::min_time) {
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
        filter.filter->FreezeUntil(last_monotonics_[node_index]);
        filter.filter->FreezeUntilRemote(last_monotonics_[filter.b_index]);
      }
      ++node_index;
    }
  }

  if (fp_) {
    fprintf(
        fp_, "%.9f",
        chrono::duration<double>(last_distributed_.time_since_epoch()).count());
    for (const monotonic_clock::time_point t : last_monotonics_) {
      fprintf(fp_, ", %.9f",
              chrono::duration<double>(t.time_since_epoch()).count());
    }
    fprintf(fp_, "\n");
  }

  return std::make_tuple(last_distributed_, last_monotonics_);
}

}  // namespace message_bridge
}  // namespace aos
