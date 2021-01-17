#ifndef AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
#define AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_

#include <functional>
#include <map>
#include <string_view>

#include "Eigen/Dense"
#include "aos/configuration.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"

namespace aos {
namespace message_bridge {

// A condensed representation of the time estimation problem statement.  This is
// designed to not have the concept of a Node object, or anything, just
// measurement pairs and indices.
//
// The problem is defined to be the squared error between the offset computed
// using packets from one node to another, and the corresponding difference in
// time between the pair of nodes.  This handles connections with data in 1
// direction and connections with data in both.
//
// All solved times are relative to the times in base_clock, and all math is
// done such that large values of base_clock don't contribute to numerical
// precision problems as long as the resulting time is small.
//
// To make this object reusable, it has a solution_node index which specifies
// which of the nodes is treated as an input and not solved for.
class TimestampProblem {
 public:
  TimestampProblem(size_t count);

  size_t size() const { return base_clock_.size(); }

  // Sets node to fix time for and not solve for.
  void set_solution_node(size_t solution_node) {
    solution_node_ = solution_node;
  }
  size_t solution_node() const { return solution_node_; }

  // Sets and gets the base time for a node.
  void set_base_clock(size_t i, monotonic_clock::time_point t) {
    base_clock_[i] = t;
  }
  monotonic_clock::time_point base_clock(size_t i) const {
    return base_clock_[i];
  }

  // Adds a timestamp filter from a -> b.
  //   filter[a_index]->Offset(ta) + ta => t(b_index);
  void add_filter(size_t a_index, const NoncausalTimestampFilter *filter,
                  size_t b_index) {
    filters_[a_index].emplace_back(filter, b_index);
  }

  // Solves the optimization problem phrased and returns the offsets from the
  // base clock for each node, excluding the solution node.
  std::vector<double> SolveDouble();
  // Solves the optimization problem phrased and returns the optimal time on
  // each node.
  std::vector<monotonic_clock::time_point> Solve();

  // Returns the squared error for all of the offsets.
  // time_offsets is the offsets from the base_clock for every node (in order)
  // except the solution node.  It should be one element shorter than the number
  // of nodes this problem was constructed with. grad (if non-nullptr) is the
  // place to put the current gradient and needs to be the same length as
  // time_offsets.
  double Cost(const double *time_offsets, double *grad);

  // Returns the time offset from base for a node.
  double get_t(const double *time_offsets, size_t node_index) const {
    if (node_index == solution_node_) return 0.0;
    size_t mapped_index = NodeToSolutionIndex(node_index);
    if (mapped_index == std::numeric_limits<size_t>::max()) {
      return 0.0;
    }
    return time_offsets[mapped_index];
  }

  // Converts a list of solutions to the corresponding monotonic times for all
  // nodes, not just the nodes being solved.
  std::vector<monotonic_clock::time_point> DoubleToMonotonic(
      const double *r) const;

  // Validates the solution, returning true if it meets all the constraints, and
  // false otherwise.
  bool ValidateSolution(std::vector<monotonic_clock::time_point> solution);

  // LOGs a representation of the problem.
  void Debug();

  // A live node is a node which has valid observations and participates in the
  // optimization problem.  Any nodes marked non-live won't be solved for.
  bool live(size_t index) const { return live_[index]; }
  void set_live(size_t index, bool live) {
    node_mapping_valid_ = false;
    live_[index] = live;
  }

  // Returns the number of live nodes.
  size_t LiveNodesCount() const {
    size_t count = 0;
    for (bool live : live_) {
      if (live) ++count;
    }
    return count;
  }

 private:
  // Static trampoline for nlopt.  n is the number of constraints, time_offsets
  // is input solution to solve for, grad is the gradient to fill out (if not
  // nullptr), and data is an untyped pointer to a TimestampProblem.
  static double DoCost(unsigned n, const double *time_offsets, double *grad,
                       void *data) {
    CHECK_EQ(n + 1u,
             reinterpret_cast<TimestampProblem *>(data)->filters_.size());
    return reinterpret_cast<TimestampProblem *>(data)->Cost(time_offsets, grad);
  }

  void MaybeUpdateNodeMapping() {
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
    node_mapping_valid_ = true;
  }

  // Converts from a node index to an index in the solution.
  size_t NodeToSolutionIndex(size_t node_index) const {
    CHECK(node_mapping_valid_);
    CHECK_NE(node_index, solution_node_);
    // The solver is going to provide us a matrix with solution_node_ removed.
    // The indices of all nodes before solution_node_ are in the same spot, and
    // the indices of the nodes after solution node are shifted over.
    size_t mapped_node_index = node_mapping_[node_index];
    return node_index < solution_node_ ? mapped_node_index
                                       : (mapped_node_index - 1);
  }

  // Number of times Cost has been called for tracking.
  int cost_call_count_ = 0;

  // The node to hold fixed when solving.
  size_t solution_node_ = 0;

  // The optimization problem is solved as base_clock + time_offsets to minimize
  // numerical precision problems.  This contains all the base times.  The base
  // time corresponding to solution_node is fixed and not solved.
  std::vector<monotonic_clock::time_point> base_clock_;
  std::vector<bool> live_;

  bool node_mapping_valid_ = false;
  std::vector<size_t> node_mapping_;

  // Filter and the node index it is referencing.
  //   filter->Offset(ta) + ta => t_(b_node);
  struct FilterPair {
    FilterPair(const NoncausalTimestampFilter *my_filter, size_t my_b_index)
        : filter(my_filter), b_index(my_b_index) {}
    const NoncausalTimestampFilter *const filter;
    const size_t b_index;
  };

  // List of filters indexed by node.
  std::vector<std::vector<FilterPair>> filters_;
};

// Helpers to convert times between the monotonic and distributed clocks for
// multiple nodes using a list of points and interpolation.
class InterpolatedTimeConverter : public TimeConverter {
 public:
  static constexpr std::chrono::nanoseconds kDefaultHistoryDuration{
      10000000000};
  static constexpr int kHistoryMinCount{500};

  InterpolatedTimeConverter(
      size_t node_count,
      std::chrono::nanoseconds time_estimation_buffer_seconds =
          kDefaultHistoryDuration)
      : node_count_(node_count),
        time_estimation_buffer_seconds_(time_estimation_buffer_seconds) {}

  virtual ~InterpolatedTimeConverter() {}

  // Converts a time to the distributed clock for scheduling and cross-node
  // time measurement.
  distributed_clock::time_point ToDistributedClock(
      size_t node_index, monotonic_clock::time_point time) override;

  // Takes the distributed time and converts it to the monotonic clock for this
  // node.
  monotonic_clock::time_point FromDistributedClock(
      size_t node_index, distributed_clock::time_point time) override;

 private:
  // Returns the next timestamp, or nullopt if there isn't one. It is assumed
  // that if there isn't one, there never will be one.
  // A timestamp is a sample of the distributed clock and a corresponding point
  // on every monotonic clock for all the nodes in the factory that this will be
  // hooked up to.
  virtual std::optional<std::tuple<distributed_clock::time_point,
                                   std::vector<monotonic_clock::time_point>>>
  NextTimestamp() = 0;

  // Queues timestamps util the last time in the queue matches the provided
  // function.
  void QueueUntil(
      std::function<
          bool(const std::tuple<distributed_clock::time_point,
                                std::vector<monotonic_clock::time_point>> &)>
          not_done);

  // The number of nodes to enforce.
  const size_t node_count_;

  // List of timestamps.
  std::deque<std::tuple<distributed_clock::time_point,
                        std::vector<monotonic_clock::time_point>>>
      times_;

  // If true, we have popped data from times_, so anything before the start is
  // unknown.
  bool have_popped_ = false;

  // The amount of time to buffer when estimating.  We care so we don't throw
  // data out of our queue too soon.  This time is indicative of how much to
  // buffer everywhere, so let's latch onto it as well until proven that there
  // is a different metric.
  const std::chrono::nanoseconds time_estimation_buffer_seconds_;

 protected:
  // If true, NextTimestamp returned nothing, so don't bother checking again.
  // (This also enforces that we don't find more time data after calling it
  // quits.)
  bool at_end_ = false;
};

enum class TimeComparison { kBefore, kAfter, kInvalid, kEq };

// Compares two times.
TimeComparison CompareTimes(const std::vector<monotonic_clock::time_point> &ta,
                            const std::vector<monotonic_clock::time_point> &tb);

// Returns the maximum amount of elapsed time between the two samples in time.
std::chrono::nanoseconds MaxElapsedTime(
    const std::vector<monotonic_clock::time_point> &ta,
    const std::vector<monotonic_clock::time_point> &tb);

// Class to hold a NoncausalOffsetEstimator per pair of communicating nodes, and
// to estimate and set the overall time of all nodes.
//
// The basic idea is that we support freezing and exploding on invalid time
// changes, but don't support unwinding time to recompute those.  This only
// works if timestamps are queued far enough in the future that we can estimate
// time far enough in the future that the current time estimate doesn't need to
// change.  The --time_estimation_buffer_seconds flag lets the user configure
// how far ahead to look, and the noncausal filters are pretty good about
// generating a couple of points a second so there is no need to adjust time.
//
// We then use lazy evaluation from the SimulatedEventLoop using the
// TimeConverter interface to trigger the oldest time to be found and added.
// This lets us evaluate time as late as possible.
//
// Each node has a list of filters, and each of those filters has an oldest
// unprocessed point on its polyline.  So, for each node, we can solve the time
// estimation problem for the oldest timestamp, find the oldest solution of all
// the nodes, add that to the InterpolatedTimeConverter polyline, and consume
// the point.
//
// TODO(austin): The event scheduler is going to ask for the time on each node
// for sorting purposes.  A node may be arbitrarily in the future, but we only
// need to compare, not return an accurate result.  If this breaks something,
// fix it.
class MultiNodeNoncausalOffsetEstimator final
    : public InterpolatedTimeConverter {
 public:
  MultiNodeNoncausalOffsetEstimator(
      SimulatedEventLoopFactory *event_loop_factory,
      const Configuration *logged_configuration, bool skip_order_validation,
      std::chrono::nanoseconds time_estimation_buffer_seconds);

  ~MultiNodeNoncausalOffsetEstimator() override;

  std::optional<std::tuple<distributed_clock::time_point,
                           std::vector<monotonic_clock::time_point>>>
  NextTimestamp() override;

  // Checks that all the nodes in the graph are connected.  Needs all filters to
  // be constructed first.
  void CheckGraph();

  // Returns the filter for a pair of nodes.  The same filter will be returned
  // for a pair of nodes, regardless of argument order.
  message_bridge::NoncausalOffsetEstimator *GetFilter(const Node *node_a,
                                                      const Node *node_b);

  // Captures the start time.
  void Start(SimulatedEventLoopFactory *factory);

  // Returns the number of nodes.
  size_t NodesCount() const {
    return configuration::NodesCount(logged_configuration());
  }

  // Returns the configuration that was logged.
  const aos::Configuration *logged_configuration() const {
    return logged_configuration_;
  }

  // Returns the configuration that we are replaying into.
  const aos::Configuration *configuration() const {
    return event_loop_factory_->configuration();
  }

 private:
  TimestampProblem MakeProblem();

  std::tuple<NoncausalTimestampFilter *,
             std::vector<aos::monotonic_clock::time_point>>
  NextSolution(TimestampProblem *problem,
               const std::vector<aos::monotonic_clock::time_point> &base_times);

  SimulatedEventLoopFactory *event_loop_factory_;
  const Configuration *logged_configuration_;

  // If true, skip any validation which would trigger if we see evidance that
  // time estimation between nodes was incorrect.
  const bool skip_order_validation_;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::map<std::tuple<const Node *, const Node *>, NoncausalOffsetEstimator>
      filters_;

  // Filter and the node index it is referencing.
  //   filter->Offset(ta) + ta => t_(b_node);
  struct FilterPair {
    FilterPair(NoncausalTimestampFilter *my_filter, size_t my_b_index)
        : filter(my_filter), b_index(my_b_index) {}
    NoncausalTimestampFilter *const filter;
    const size_t b_index;
  };
  std::vector<std::vector<FilterPair>> filters_per_node_;

  distributed_clock::time_point last_distributed_ = distributed_clock::epoch();
  std::vector<aos::monotonic_clock::time_point> last_monotonics_;

  bool first_solution_ = true;
  bool all_done_ = false;

  FILE *fp_ = NULL;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
