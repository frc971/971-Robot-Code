#ifndef AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
#define AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_

#include <functional>
#include <map>
#include <string_view>

#include "Eigen/Dense"
#include "absl/container/btree_set.h"
#include "aos/configuration.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/logging/logfile_utils.h"
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
  void set_base_clock(size_t i, logger::BootTimestamp t) { base_clock_[i] = t; }
  logger::BootTimestamp base_clock(size_t i) const { return base_clock_[i]; }

  // Adds a timestamp filter from a -> b.
  //   filter[a_index]->Offset(ta) + ta => t(b_index);
  void add_filter(size_t a_index, const NoncausalTimestampFilter *filter,
                  size_t b_index) {
    filters_[a_index].emplace_back(filter, b_index);
  }

  // Solves the optimization problem phrased using the symmetric Netwon's method
  // solver and returns the optimal time on each node.
  std::vector<logger::BootTimestamp> SolveNewton();

  // Validates the solution, returning true if it meets all the constraints, and
  // false otherwise.
  bool ValidateSolution(std::vector<logger::BootTimestamp> solution);

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
  // Returns the Hessian of the cost function at time_offsets.
  Eigen::MatrixXd Hessian(const Eigen::Ref<Eigen::VectorXd> time_offsets) const;
  // Returns the gradient of the cost function at time_offsets.
  Eigen::VectorXd Gradient(
      const Eigen::Ref<Eigen::VectorXd> time_offsets) const;

  // Returns the newton step of the timestamp problem.  The last term is the
  // scalar on the equality constraint.  This needs to be removed from the
  // solution to get the actual newton step.
  Eigen::VectorXd Newton(const Eigen::Ref<Eigen::VectorXd> time_offsets) const;

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
    live_nodes_ = live_node_index;
    node_mapping_valid_ = true;
  }

  // Converts from a node index to an index in the solution without skipping the
  // solution node.
  size_t NodeToFullSolutionIndex(size_t node_index) const {
    CHECK(node_mapping_valid_);
    return node_mapping_[node_index];
  }

  // The node to hold fixed when solving.
  size_t solution_node_ = 0;

  // The optimization problem is solved as base_clock + time_offsets to minimize
  // numerical precision problems.  This contains all the base times.  The base
  // time corresponding to solution_node is fixed and not solved.
  std::vector<logger::BootTimestamp> base_clock_;
  std::vector<bool> live_;

  // True if both node_mapping_ and live_nodes_ are valid.
  bool node_mapping_valid_ = false;
  // Mapping from a node index to an index in the solution.
  std::vector<size_t> node_mapping_;
  // The number of live nodes there are.
  size_t live_nodes_ = 0;

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
      size_t node_index, logger::BootTimestamp time) override;

  // Takes the distributed time and converts it to the monotonic clock for this
  // node.
  logger::BootTimestamp FromDistributedClock(size_t node_index,
                                             distributed_clock::time_point time,
                                             size_t boot_count) override;

  // Called whenever time passes this point and we can forget about it.
  // TODO(austin): Pop here instead of in log reader.
  void ObserveTimePassed(distributed_clock::time_point time) override;

  // Queues 1 more timestammp in the interpolation list.  This is public for
  // timestamp_extractor so it can hammer on the log until everything is queued.
  std::optional<const std::tuple<distributed_clock::time_point,
                                 std::vector<logger::BootTimestamp>> *>
  QueueNextTimestamp();

 private:
  // Returns the next timestamp, or nullopt if there isn't one. It is assumed
  // that if there isn't one, there never will be one.
  // A timestamp is a sample of the distributed clock and a corresponding point
  // on every monotonic clock for all the nodes in the factory that this will be
  // hooked up to.
  virtual std::optional<std::tuple<distributed_clock::time_point,
                                   std::vector<logger::BootTimestamp>>>
  NextTimestamp() = 0;

  // Queues timestamps util the last time in the queue matches the provided
  // function.
  void QueueUntil(
      std::function<
          bool(const std::tuple<distributed_clock::time_point,
                                std::vector<logger::BootTimestamp>> &)>
          not_done);

  // The number of nodes to enforce.
  const size_t node_count_;

 protected:
  // List of timestamps.
  std::deque<std::tuple<distributed_clock::time_point,
                        std::vector<logger::BootTimestamp>>>
      times_;

  // If true, we have popped data from times_, so anything before the start is
  // unknown.
  bool have_popped_ = false;

  // The amount of time to buffer when estimating.  We care so we don't throw
  // data out of our queue too soon.  This time is indicative of how much to
  // buffer everywhere, so let's latch onto it as well until proven that there
  // is a different metric.
  const std::chrono::nanoseconds time_estimation_buffer_seconds_;

  // If true, NextTimestamp returned nothing, so don't bother checking again.
  // (This also enforces that we don't find more time data after calling it
  // quits.)
  bool at_end_ = false;
};

enum class TimeComparison { kBefore, kAfter, kInvalid, kEq };

// Compares two sets of times, optionally ignoring times that are min_time
TimeComparison CompareTimes(const std::vector<logger::BootTimestamp> &ta,
                            const std::vector<logger::BootTimestamp> &tb);

// Returns the maximum amount of elapsed time between the two samples in time.
std::chrono::nanoseconds MaxElapsedTime(
    const std::vector<logger::BootTimestamp> &ta,
    const std::vector<logger::BootTimestamp> &tb);

// Returns the amount of time by which ta and tb are out of order.  The primary
// direction is defined to be the direction of the average of the offsets.  So,
// if the average is +, and we get a -ve outlier, the absolute value of that -ve
// outlier is the invalid distance.
std::chrono::nanoseconds InvalidDistance(
    const std::vector<logger::BootTimestamp> &ta,
    const std::vector<logger::BootTimestamp> &tb);

// Interpolates a monotonic time to a distributed time without loss of
// precision.  Implements (d1 - d0) / (t1 - t0) * (time - t0) + d0;
distributed_clock::time_point ToDistributedClock(
    distributed_clock::time_point d0, distributed_clock::time_point d1,
    monotonic_clock::time_point t0, monotonic_clock::time_point t1,
    monotonic_clock::time_point time);

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
      const Configuration *configuration,
      const Configuration *logged_configuration,
      std::shared_ptr<const logger::Boots> boots, bool skip_order_validation,
      std::chrono::nanoseconds time_estimation_buffer_seconds);

  ~MultiNodeNoncausalOffsetEstimator() override;

  // Sets the timestamp mappers for all the nodes.  This registers the timestamp
  // callback to add elements to the filters.
  void SetTimestampMappers(
      std::vector<logger::TimestampMapper *> timestamp_mappers);

  std::optional<std::tuple<distributed_clock::time_point,
                           std::vector<logger::BootTimestamp>>>
  NextTimestamp() override;

  UUID boot_uuid(size_t node_index, size_t boot_count) override;

  // Checks that all the nodes in the graph are connected.  Needs all filters to
  // be constructed first.
  void CheckGraph();

  // Returns the filter for a pair of nodes.  The same filter will be returned
  // for a pair of nodes, regardless of argument order.
  message_bridge::NoncausalOffsetEstimator *GetFilter(const Node *node_a,
                                                      const Node *node_b);

  // Captures the start time.
  void Start(SimulatedEventLoopFactory *factory);
  void Start(std::vector<monotonic_clock::time_point> times);

  // Returns the number of nodes.
  size_t NodesCount() const {
    return configuration::NodesCount(logged_configuration());
  }

  // Returns the configuration that was logged.
  const aos::Configuration *logged_configuration() const {
    return logged_configuration_;
  }

  // Returns the configuration that we are replaying into.
  const aos::Configuration *configuration() const { return configuration_; }

 private:
  TimestampProblem MakeProblem();

  // Returns the next solution, the filter which has the control point for it
  // (or nullptr if a start time triggered this to be returned), and the node
  // which triggered it.
  std::tuple<NoncausalTimestampFilter *, std::vector<logger::BootTimestamp>,
             int>
  NextSolution(TimestampProblem *problem,
               const std::vector<logger::BootTimestamp> &base_times);

  // Writes all samples to disk.
  void FlushAllSamples(bool finish);

  const Configuration *configuration_;
  const Configuration *logged_configuration_;

  std::shared_ptr<const logger::Boots> boots_;

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
  std::vector<logger::BootTimestamp> last_monotonics_;

  // A mapping from node and channel to the relevant estimator.
  std::vector<std::vector<NoncausalOffsetEstimator *>> filters_per_channel_;

  std::vector<logger::TimestampMapper *> timestamp_mappers_;

  bool first_solution_ = true;
  bool all_done_ = false;

  // Optional file pointers to save the results of the noncausal filter in. This
  // lives here so we can give each sample a distributed clock.
  std::vector<std::vector<FILE *>> filter_fps_;
  // Optional file pointers to save all the samples into.
  std::vector<std::vector<FILE *>> sample_fps_;

  FILE *fp_ = NULL;

  struct SingleNodeSamples {
    struct CompareTimestamps {
      bool operator()(
          const std::pair<logger::BootTimestamp, logger::BootTimestamp> &a,
          const std::pair<logger::BootTimestamp, logger::BootTimestamp> &b)
          const {
        return a.first < b.first;
      }
    };

    // Delivered, sent timestamps for each message.
    absl::btree_set<std::pair<logger::BootTimestamp, logger::BootTimestamp>,
                    CompareTimestamps>
        messages;
  };

  struct NodeSamples {
    // List of nodes sending.
    std::vector<SingleNodeSamples> nodes;
  };

  // List of nodes where data is delivered.
  std::vector<NodeSamples> node_samples_;
  // Mapping from channel to the node_index of the source node.
  std::vector<size_t> source_node_index_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
