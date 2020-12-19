#ifndef AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
#define AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_

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

  // Returns [ta; tb; ...] = tuple[0] * t + tuple[1]
  std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
             Eigen::Matrix<double, Eigen::Dynamic, 1>>
  SolveOffsets();

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
  SimulatedEventLoopFactory *event_loop_factory_;
  const Configuration *logged_configuration_;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::map<std::tuple<const Node *, const Node *>,
           std::tuple<NoncausalOffsetEstimator>>
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
