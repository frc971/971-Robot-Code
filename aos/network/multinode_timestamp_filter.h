#ifndef AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_
#define AOS_NETWORK_MULTINODE_TIMESTAMP_FILTER_H_

#include <functional>
#include <map>
#include <string_view>

#include "Eigen/Dense"
#include "absl/container/btree_set.h"
#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"

namespace aos {
namespace message_bridge {

// Problem description for NewtonSolver.
class Problem {
 public:
  // The derivatives and other work products needed for both the constrained and
  // unconstrained newtons method.
  struct Derivatives {
    Eigen::VectorXd gradient;
    Eigen::MatrixXd hessian;

    // Inequality function f
    Eigen::MatrixXd f;
    // df
    Eigen::MatrixXd df;
    // Slope limited df. This is used as part of computing the modified KKT
    // conditions to avoid discontinuities across segment boundaries in lambda,
    // and therefor Rt.  See Rt in the .cc file for more info.
    Eigen::MatrixXd df_slope_limited;

    // ddf is assumed to be 0 because for the linear constraint distance
    // function we are using, it is actually 0, and by assuming it is zero
    // rather than passing it through as 0 to the solver, we can save enough CPU
    // to make it worth it.

    // A
    Eigen::MatrixXd A;
    // Ax - b
    Eigen::MatrixXd Axmb;
    // Node picked for Axmb
    size_t solution_node = std::numeric_limits<size_t>::max();
  };

  // Computes the derivatives of the cost function.
  // If quiet is true, ComputeDerivatives shouldn't print anything.  If all is
  // true, all inequality constraints are populated, otherwise only the
  // constraint indices (indexed into the list of all constraints) provided are
  // referenced.
  virtual Derivatives ComputeDerivatives(
      size_t solve_number, const Eigen::Ref<const Eigen::VectorXd> y,
      bool quiet, bool all,
      const absl::Span<const size_t> active_constraints) = 0;

  // Prepares the problem to be solved.  This is called when the problem is
  // started before any derivatives are computed.
  virtual void Prepare(size_t my_solve_number) = 0;

  // Observes (and potentially modifies y) for a new y.
  virtual void Update(size_t solve_number, Eigen::Ref<Eigen::VectorXd> y) = 0;

  // Returns the number of states.
  size_t states() const { return states_; }

  // Returns the number of inequality constraints.
  size_t inequality_constraints() const { return inequality_constraints_; }

 protected:
  size_t states_ = 0;
  size_t inequality_constraints_ = 0;
};

// Implements a solver for various Problems.
class NewtonSolver {
 public:
  // Ratio to let the cost increase when line searching.  A small increase is
  // fine since we aren't perfectly convex.
  static constexpr double kAlpha = -0.15;
  // Ratio to require the cost to decrease when line searching.
  static constexpr double kUnconstrainedAlpha = 0.5;
  // Unconstrained min line search distance to guarantee forward progress.
  static constexpr double kLineSearchStopThreshold = 0.4;
  // Line search step parameter.
  static constexpr double kBeta = 0.5;
  static constexpr double kMu = 2.0;
  // Terminal condition for the primal problem (equality constraints) and dual
  // (gradient + inequality constraints).
  static constexpr double kEpsilonF = 1e-4;
  // Terminal condition for nu, the surrogate duality gap.
  static constexpr double kEpsilon = 1e-2;

  NewtonSolver();

  // Returns which solve number this is.  This increments each time NewtonSolver
  // is constructed, and is only used for debugging.
  size_t my_solve_number() const { return my_solve_number_; }

  // Solves a problem using an infeasible start unconstrained Newtons method
  // solver.  There is no line search.
  //
  // minimize     f0(x)
  //  subject to  A x = 0
  //
  // Returns the solution, the solution_node field from the final derivatives,
  // and the number of iterations it took
  std::tuple<Eigen::VectorXd, size_t, size_t> SolveNewton(
      Problem *problem, const size_t max_iterations);

  // Solves a problem using an infeasible start  Primal-dual interior-point
  // method solver. Only respects the constraint indices provided and which are
  // violated by the initial x(0).
  //
  // minimize     f0(x)
  //  subject to  f(x) <=0
  //              A x = 0
  //
  // Returns the solution, the solution_node field from the final derivatives,
  // the number of iterations it took, and a list of the constraints actually
  // used in the solution.
  std::optional<
      std::tuple<Eigen::VectorXd, size_t, size_t, std::vector<size_t>>>
  SolveConstrainedNewton(Problem *problem, size_t max_iterations,
                         const absl::Span<const size_t> constraint_indices);

  // Adds our slack variable to our constrained problem derivatives.
  static Problem::Derivatives AddConstraintSlackVariable(
      const Problem::Derivatives &derivatives,
      const Eigen::Ref<const Eigen::VectorXd> y);

 private:
  // Returns the modified KKT conditions
  // (11.53 in Convex Optimization,
  //  https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf)
  Eigen::VectorXd Rt(const Problem::Derivatives &derivatives,
                     Eigen::Ref<const Eigen::VectorXd> y, double t);

  // Returns the squared norm of r for the unconstrained problem.
  // (10.3.1 in Convex Optimization,
  //  https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf)
  double RSquaredUnconstrained(const Problem::Derivatives &derivatives,
                               Eigen::Ref<const Eigen::VectorXd> y,
                               Eigen::Ref<const Eigen::VectorXd> step,
                               double t);

  // Returns the constrained newtons step, t_inverse, and Rt.
  std::tuple<Eigen::VectorXd, double, Eigen::VectorXd> ConstrainedNewton(
      const Eigen::Ref<const Eigen::VectorXd> y,
      const Problem::Derivatives &derivatives, size_t iteration);

  // Prints out the provided derivatives for debugging.
  void PrintDerivatives(const Problem::Derivatives &derivatives,
                        const Eigen::Ref<const Eigen::VectorXd> y,
                        std::string_view prefix, int verbosity);

  // Returns the newton step of the timestamp problem, and the node which was
  // used for the equality constraint.  The last term is the scalar on the
  // equality constraint.  This needs to be removed from the solution to get the
  // actual newton step.
  Eigen::VectorXd Newton(const Eigen::Ref<const Eigen::VectorXd> y,
                         const Problem::Derivatives &derivatives,
                         size_t iteration);

  // The solve number for this instance of the problem.
  size_t my_solve_number_;

  // The global solve number counter used to deterministically find problems.
  static size_t solve_number_;
};

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
class TimestampProblem : public Problem {
 public:
  TimestampProblem(size_t count);

  size_t size() const { return base_clock_.size(); }

  // Sets and gets the base time for a node.
  void set_base_clock(size_t i, logger::BootTimestamp t) {
    // Valid boots is potentially invalidated here, let's just wipe our cache.
    // This won't happen during a solve.
    node_mapping_valid_ = false;
    base_clock_[i] = t;
  }
  logger::BootTimestamp base_clock(size_t i) const { return base_clock_[i]; }

  // Adds a timestamp filter from a -> b.
  //   filter[a_index]->Offset(ta) + ta => t(b_index);
  void add_clock_offset_filter(size_t a_index,
                               const NoncausalTimestampFilter *filter,
                               size_t b_index,
                               const NoncausalTimestampFilter *b_filter) {
    clock_offset_filter_for_node_[a_index].emplace_back(filter, b_index,
                                                        b_filter);
  }

  // Adds both base_clock_ and y together to produce the final answer.
  std::vector<logger::BootTimestamp> PackResults(
      bool print, const Eigen::Ref<const Eigen::VectorXd> y);

  // Validates the solution, returning true if it meets all the constraints, and
  // false otherwise.
  bool ValidateSolution(std::vector<logger::BootTimestamp> solution,
                        bool quiet);
  // Returns true if the provide node has observations to solve for the
  // provided boots.  This may happen when we are trying to solve for a reboot
  // to see if it is next, and haven't queued far enough.
  bool HasObservations(size_t node_a) const;

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

  // Sets the points to solve an equality constraint to.
  void set_points(const absl::Span<const logger::BootTimestamp> points) {
    points_ = points;
  }

  // Adds the violated constraints from result to new_constraint_indices.
  // Returns true if anything was added.
  bool AddConstraints(size_t solve_number,
                      const std::vector<logger::BootTimestamp> &result,
                      std::vector<size_t> *new_constraint_indices);

  // Returns all the derivatives of the cost function at the provide
  // time_offsets using the symmetric problem definition.  If quiet is
  // specificed, nothing is printed. If all is specified, all the constraints
  // are considered, otherwise, only the constraint indices in active_constraint
  // are considered.
  Problem::Derivatives ComputeDerivatives(
      size_t solve_number, const Eigen::Ref<const Eigen::VectorXd> time_offsets,
      bool quiet, bool all,
      const absl::Span<const size_t> active_constraints) override;

 private:
  // Returns the number of live constraints.
  size_t LiveConstraintsCount() const;

  size_t SolutionNode(const std::vector<logger::BootTimestamp> &points) const {
    size_t solution_node = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      if (points[i] != logger::BootTimestamp::max_time()) {
        CHECK_EQ(solution_node, std::numeric_limits<size_t>::max());
        solution_node = i;
      }
    }
    CHECK_NE(solution_node, std::numeric_limits<size_t>::max());
    return solution_node;
  }

  // Sets the problem up to be ready to solve.  Caches anything we need during
  // the solve.
  void Prepare(size_t my_solve_number) override;

  // Updates base_clock_ if y is too big to keep the solution small.  Updates y
  // to account for any change.
  void Update(size_t solve_number, Eigen::Ref<Eigen::VectorXd> y) override;

  void MaybeUpdateNodeMapping();

  // Converts from a node index to an index in the solution without skipping the
  // solution node.
  size_t NodeToFullSolutionIndex(size_t node_index) const {
    CHECK(node_mapping_valid_);
    return node_mapping_[node_index];
  }

  // The optimization problem is solved as base_clock + time_offsets to minimize
  // numerical precision problems.  This contains all the base times.  The base
  // time corresponding to solution_node is fixed and not solved.
  std::vector<logger::BootTimestamp> base_clock_;
  std::vector<bool> live_;

  // True if both node_mapping_ and live_nodes_ are valid.
  bool node_mapping_valid_ = false;
  // Mapping from a node index to an index in the solution.
  std::vector<size_t> node_mapping_;

  // Filter and the node index it is referencing.
  //   filter->Offset(ta) + ta => t_(b_node);
  struct FilterPair {
    FilterPair(const NoncausalTimestampFilter *my_filter, size_t my_b_index,
               const NoncausalTimestampFilter *b_filter)
        : filter(my_filter), b_index(my_b_index), b_filter(b_filter) {}
    const NoncausalTimestampFilter *const filter;
    const size_t b_index;

    const NoncausalTimestampFilter *const b_filter;

    NoncausalTimestampFilter::Pointer pointer;
  };

  // List of filters indexed by node.
  std::vector<std::vector<FilterPair>> clock_offset_filter_for_node_;

  // Points to solve equality constraints to.
  absl::Span<const logger::BootTimestamp> points_;
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

  // Queues 1 more timestamp in the interpolation list.  This is public for
  // timestamp_extractor so it can hammer on the log until everything is queued.
  // Return type matches that of NextTimestamp().
  [[nodiscard]] std::optional<std::optional<const std::tuple<
      distributed_clock::time_point, std::vector<logger::BootTimestamp>> *>>
  QueueNextTimestamp();

 private:
  // Returns the next timestamp, or nullopt if there isn't one. It is assumed
  // that if there isn't one, there never will be one.
  // A timestamp is a sample of the distributed clock and a corresponding point
  // on every monotonic clock for all the nodes in the factory that this will be
  // hooked up to.
  // If !NextTimestamp().has_value(), then an error occurred. If
  // !NextTimestamp().value().has_value(), then there is merely no next
  // timestamp available.
  // TODO(james): Swap this to std::expected when available.
  virtual std::optional<std::optional<std::tuple<
      distributed_clock::time_point, std::vector<logger::BootTimestamp>>>>
  NextTimestamp() = 0;

  // Queues timestamps util the last time in the queue matches the provided
  // function.
  template <typename F>
  void QueueUntil(F not_done) {
    while (!at_end_ && (times_.empty() || not_done(times_.back()))) {
      // Check the outer std::optional to watch for errors.
      CHECK(QueueNextTimestamp().has_value())
          << ": An error occurred when queueing timestamps.";
    }

    CHECK(!times_.empty())
        << ": Found no times to do timestamp estimation, please investigate.";
  }

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

  // Runs some checks that normally run with fatal CHECK's in the destructor.
  // Returns false if any checks failed. This is used to allow the
  // logfile_validator to non-fatally identify certain log sorting issues.
  [[nodiscard]] bool RunDestructorChecks() {
    non_fatal_destructor_checks_ = true;
    return FlushAndClose(false);
  }

 private:
  static constexpr int kMaxIterations = 400;

  struct CandidateTimes {
    logger::BootTimestamp next_node_time = logger::BootTimestamp::max_time();
    logger::BootDuration next_node_duration = logger::BootDuration::max_time();
    size_t b_index = std::numeric_limits<size_t>::max();
    NoncausalTimestampFilter *next_node_filter = nullptr;
  };

  TimestampProblem MakeProblem();

  std::optional<std::optional<std::tuple<distributed_clock::time_point,
                                         std::vector<logger::BootTimestamp>>>>
  NextTimestamp() override;

  // Returns the list of candidate times to solve for.
  std::tuple<std::vector<CandidateTimes>, bool> MakeCandidateTimes() const;

  // Returns the next solution, the filter which has the control point for it
  // (or nullptr if a start time triggered this to be returned), and the node
  // which triggered it.
  std::optional<std::tuple<NoncausalTimestampFilter *,
                           std::vector<logger::BootTimestamp>, int>>
  NextSolution(TimestampProblem *problem,
               const std::vector<logger::BootTimestamp> &base_times);

  // Returns the solution (if there is one) for the list of candidate times by
  // solving all the problems simultaneously.  They must be from the same boot.
  std::optional<std::tuple<NoncausalTimestampFilter *,
                           std::vector<logger::BootTimestamp>, int>>
  SimultaneousSolution(TimestampProblem *problem,
                       const std::vector<CandidateTimes> candidate_times,
                       const std::vector<logger::BootTimestamp> &base_times);

  // Returns the solution (if there is one) for the list of candidate times by
  // solving the problems one after another.  They can be from any boot.
  std::optional<std::tuple<NoncausalTimestampFilter *,
                           std::vector<logger::BootTimestamp>, int>>
  SequentialSolution(TimestampProblem *problem,
                     const std::vector<CandidateTimes> candidate_times,
                     const std::vector<logger::BootTimestamp> &base_times);

  // Returns false if the invalid distance is too far.
  [[nodiscard]] bool CheckInvalidDistance(
      const std::vector<logger::BootTimestamp> &result_times,
      const std::vector<logger::BootTimestamp> &solution);

  // Writes all samples to disk.
  void FlushAllSamples(bool finish);

  // Adds the solution to last_distributed_.
  void UpdateSolution(std::vector<logger::BootTimestamp> result_times);

  void WriteFilter(
      NoncausalTimestampFilter *next_filter,
      std::tuple<logger::BootTimestamp, logger::BootDuration> sample);

  // Writes everything to disk anc closes it all out in preparation for either
  // destruction or crashing.
  // Returns true if everything is healthy; returns false if we discovered
  // sorting issues when closing things out.
  [[nodiscard]] bool FlushAndClose(bool destructor);

  const Configuration *configuration_;
  const Configuration *logged_configuration_;

  std::shared_ptr<const logger::Boots> boots_;

  // If true, don't fatally die on any order validation failures which would
  // trigger if we see evidence that time estimation between nodes was
  // incorrect.
  const bool skip_order_validation_;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::map<std::tuple<const Node *, const Node *>, NoncausalOffsetEstimator>
      filters_;

  // Filter and the node index it is referencing.
  //   filter->Offset(ta) + ta => t_(b_node);
  struct FilterPair {
    FilterPair(NoncausalTimestampFilter *my_filter, size_t my_b_index,
               NoncausalTimestampFilter *b_filter)
        : filter(my_filter), b_index(my_b_index), b_filter(b_filter) {}
    NoncausalTimestampFilter *const filter;
    const size_t b_index;
    NoncausalTimestampFilter *const b_filter;
  };
  std::vector<std::vector<FilterPair>> filters_per_node_;

  distributed_clock::time_point last_distributed_ = distributed_clock::epoch();
  std::vector<logger::BootTimestamp> last_monotonics_;

  // A mapping from node and channel to the relevant estimator.
  std::vector<std::vector<NoncausalOffsetEstimator *>> filters_per_channel_;

  std::vector<logger::TimestampMapper *> timestamp_mappers_;

  bool first_solution_ = true;
  bool all_done_ = false;
  bool non_fatal_destructor_checks_ = false;

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
