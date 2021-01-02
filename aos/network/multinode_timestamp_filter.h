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
#include "third_party/gmp/gmpxx.h"

namespace aos {
namespace message_bridge {

// A condensed representation of the time estimation problem statement.  This is
// designed to not have the concept of a Node object, or anything, just
// measurement pairs and indices.
//
// The problem is defined to be the squared error between the offset computed
// using packets from one node to another, and the corresponding difference in
// time the pair of node.  This handles connections with data in 1 direction and
// connections with data in both.
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
  std::vector<double> Solve();

  // Returns the squared error for all of the offsets.
  // x is the offsets from the base_clock for every node (in order) except the
  // solution node.  It should be one element shorter than the number of nodes
  // this problem was constructed with.
  // grad (if non-nullptr) is the place to put the current gradient and needs to
  // be the same length as x.
  double Cost(const double *x, double *grad);

  // Returns the time offset from base for a node.
  double get_t(const double *x, size_t time_index) {
    return time_index == solution_node_ ? 0.0
                                       : x[NodeToSolutionIndex(time_index)];
  }

  // LOGs a representation of the problem.
  void Debug();

 private:
  // Static trampoline for nlopt.  n is the number of constraints, x is input
  // solution to solve for, grad is the gradient to fill out (if not nullptr),
  // and data is an untyped pointer to a TimestampProblem.
  static double DoCost(unsigned n, const double *x, double *grad, void *data) {
    CHECK_EQ(n + 1u,
             reinterpret_cast<TimestampProblem *>(data)->filters_.size());
    return reinterpret_cast<TimestampProblem *>(data)->Cost(x, grad);
  }

  // Converts from a node index to an index in the solution.
  size_t NodeToSolutionIndex(size_t node_index) {
    // The solver is going to provide us a matrix with solution_node_ removed.
    // The indices of all nodes before solution_node_ are in the same spot, and
    // the indices of the nodes after solution node are shifted over.
    return node_index < solution_node_ ? node_index : (node_index - 1);
  }

  // Number of times Cost has been called for tracking.
  int count_ = 0;

  // The node to hold fixed when solving.
  size_t solution_node_ = 0;

  // The optimization problem is solved as base_clock + x to minimize numerical
  // precision problems.  This contains all the base times.  The base time
  // corresponding to solution_node is fixed and not solved.
  std::vector<monotonic_clock::time_point> base_clock_;

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
  InterpolatedTimeConverter(size_t node_count) : node_count_(node_count) {}

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

 protected:
  // If true, NextTimestamp returned nothing, so don't bother checking again.
  // (This also enforces that we don't find more time data after calling it
  // quits.)
  bool at_end_ = false;
};

// Class to hold a NoncausalOffsetEstimator per pair of communicating nodes, and
// to estimate and set the overall time of all nodes.
class MultiNodeNoncausalOffsetEstimator {
 public:
  MultiNodeNoncausalOffsetEstimator(
      SimulatedEventLoopFactory *event_loop_factory)
      : event_loop_factory_(event_loop_factory) {}

  // Constructs all the matricies.  Needs to be called after the log files have
  // been opened and all the filters have been created.
  void Initialize(const Configuration *logged_configuration);

  // Returns the filter for a pair of nodes.  The same filter will be returned
  // for a pair of nodes, regardless of argument order.
  message_bridge::NoncausalOffsetEstimator *GetFilter(const Node *node_a,
                                                      const Node *node_b);

  // Prints out debug information about the line segments for each node.
  void LogFit(std::string_view prefix);

  // Captures the start time.
  void Start(SimulatedEventLoopFactory *factory);

  // Returns the number of nodes.
  size_t nodes_count() const {
    return !configuration::MultiNode(logged_configuration())
               ? 1u
               : logged_configuration()->nodes()->size();
  }

  // Returns the offset from the monotonic clock for a node to the distributed
  // clock.  monotonic = distributed * slope() + offset();
  double slope(int node_index) const {
    CHECK_LT(node_index, time_slope_matrix_.rows())
        << ": Got too high of a node index.";
    return time_slope_matrix_(node_index);
  }
  std::chrono::nanoseconds offset(int node_index) const {
    CHECK_LT(node_index, time_offset_matrix_.rows())
        << ": Got too high of a node index.";
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(time_offset_matrix_(node_index)));
  }

  // Returns the configuration that was logged.
  const aos::Configuration *logged_configuration() const {
    return logged_configuration_;
  }

  // Returns the configuration that we are replaying into.
  const aos::Configuration *configuration() const {
    return event_loop_factory_->configuration();
  }

  // Returns the number of nodes connected to each other.
  size_t ConnectedNodes();

  // Recomputes the offsets and sets them on event_loop_factory_.
  void UpdateOffsets();

 private:
  // Returns [ta; tb; ...] = tuple[0] * t + tuple[1]
  std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
             Eigen::Matrix<double, Eigen::Dynamic, 1>>
  SolveOffsets();

  SimulatedEventLoopFactory *event_loop_factory_;
  const Configuration *logged_configuration_;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::map<std::tuple<const Node *, const Node *>, NoncausalOffsetEstimator>
      filters_;

  // We have 2 types of equations to do a least squares regression over to fully
  // constrain our time function.
  //
  // One is simple.  The distributed clock is the average of all the clocks.
  //   (ta + tb + tc + td) / num_nodes = t_distributed
  //
  // The second is a bit more complicated.  Our basic time conversion function
  // is:
  //   tb = ta + (ta * slope + offset)
  // We can rewrite this as follows
  //   tb - (1 + slope) * ta = offset
  //
  // From here, we have enough equations to solve for t{a,b,c,...}  We want to
  // take as an input the offsets and slope, and solve for the per-node times as
  // a function of the distributed clock.
  //
  // We need to massage our equations to make this work.  If we solve for the
  // per-node times at two set distributed clock times, we will be able to
  // recreate the linear function (we know it is linear).  We can do a similar
  // thing by breaking our equation up into:
  //
  // [1/3  1/3  1/3  ] [ta]   [t_distributed]
  // [ 1  -1-m1  0   ] [tb] = [oab]
  // [ 1    0  -1-m2 ] [tc]   [oac]
  //
  // This solves to:
  //
  // [ta]   [ a00 a01 a02]   [t_distributed]
  // [tb] = [ a10 a11 a12] * [oab]
  // [tc]   [ a20 a21 a22]   [oac]
  //
  // and can be split into:
  //
  // [ta]   [ a00 ]                   [a01 a02]
  // [tb] = [ a10 ] * t_distributed + [a11 a12] * [oab]
  // [tc]   [ a20 ]                   [a21 a22]   [oac]
  //
  // (map_matrix_ + slope_matrix_) * [ta; tb; tc] = [offset_matrix_];
  // offset_matrix_ will be in nanoseconds.
  Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> map_matrix_;
  Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> slope_matrix_;
  Eigen::Matrix<mpq_class, Eigen::Dynamic, 1> offset_matrix_;
  // Matrix tracking which offsets are valid.
  Eigen::Matrix<bool, Eigen::Dynamic, 1> valid_matrix_;
  // Matrix tracking the last valid matrix we used to determine connected nodes.
  Eigen::Matrix<bool, Eigen::Dynamic, 1> last_valid_matrix_;
  size_t cached_valid_node_count_ = 0;

  // [ta; tb; tc] = time_slope_matrix_ * t + time_offset_matrix;
  // t is in seconds.
  Eigen::Matrix<double, Eigen::Dynamic, 1> time_slope_matrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> time_offset_matrix_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
