#include "aos/network/multinode_timestamp_filter.h"

#include <chrono>
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

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ToDouble(
    Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> in) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> result =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(in.rows(),
                                                                  in.cols());
  for (int i = 0; i < in.rows(); ++i) {
    for (int j = 0; j < in.cols(); ++j) {
      result(i, j) = in(i, j).get_d();
    }
  }
  return result;
}

std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
           Eigen::Matrix<double, Eigen::Dynamic, 1>>
Solve(const Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> &mpq_map,
      const Eigen::Matrix<mpq_class, Eigen::Dynamic, 1> &mpq_offsets) {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();
  // Least squares solve for the slopes and offsets.
  const Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> inv =
      (mpq_map.transpose() * mpq_map).inverse() * mpq_map.transpose();
  aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  VLOG(3) << "Took "
          << std::chrono::duration<double>(end_time - start_time).count()
          << " seconds to invert";

  Eigen::Matrix<mpq_class, Eigen::Dynamic, 1> mpq_solution_slope =
      inv.block(0, 0, inv.rows(), 1);
  Eigen::Matrix<mpq_class, Eigen::Dynamic, 1> mpq_solution_offset =
      inv.block(0, 1, inv.rows(), inv.cols() - 1) *
      mpq_offsets.block(1, 0, inv.rows() - 1, 1);

  mpq_solution_offset *= mpq_class(1, 1000000000);

  return std::make_tuple(ToDouble(mpq_solution_slope),
                         ToDouble(mpq_solution_offset));
}
}  // namespace

TimestampProblem::TimestampProblem(size_t count) {
  CHECK_GT(count, 1u);
  filters_.resize(count);
  base_clock_.resize(count);
}

// TODO(austin): Adjust simulated event loop factory to take lists of points
// on all clocks and interpolate.
//
// TODO(austin): Add linear inequality constraints too.
//
// TODO(austin): Add a rate of change constraint from the last sample.  1
// ms/s.  Figure out how to define it.  Do this last.  This lets us handle
// constraints going away, and constraints close in time.

std::vector<double> TimestampProblem::Solve() {
  // TODO(austin): Add constraints for relevant segments.
  const size_t n = filters_.size() - 1u;
  //  NLOPT_LD_MMA and NLOPT_LD_LBFGS are alternative solvers, but SLSQP is a
  //  better fit for the quadratic nature of this problem.
  nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, n);
  nlopt_set_min_objective(opt, TimestampProblem::DoCost, this);

  // Ask for really good.  This is very quadratic, so it should be pretty
  // precise.
  nlopt_set_xtol_rel(opt, 1e-19);

  count_ = 0;

  std::vector<double> result(n, 0.0);
  double minf = 0.0;
  CHECK_GE(nlopt_optimize(opt, result.data(), &minf), 0);

  if (VLOG_IS_ON(1)) {
    std::vector<double> gradient(n, 0.0);
    Cost(result.data(), gradient.data());

    // High precision formatter for the gradient.
    struct MyFormatter {
      void operator()(std::string *out, double i) const {
        std::stringstream ss;
        ss << std::setprecision(12) << std::fixed << i;
        out->append(ss.str());
      }
    };

    VLOG(1) << std::setprecision(12) << std::fixed << "Found minimum at f("
            << result[0] << ") -> " << minf << " grad ["
            << absl::StrJoin(gradient, ", ", MyFormatter()) << "] after "
            << count_ << " cycles.";
  }
  return result;
}

double TimestampProblem::Cost(const double *x, double *grad) {
  ++count_;

  if (grad != nullptr) {
    for (size_t i = 0; i < filters_.size() - 1u; ++i) {
      grad[i] = 0;
    }

    for (size_t i = 0u; i < filters_.size(); ++i) {
      for (const struct FilterPair &filter : filters_[i]) {
        if (i != solution_node_) {
          grad[NodeToSolutionIndex(i)] += filter.filter->DCostDta(
              base_clock_[i], get_t(x, i), base_clock_[filter.b_index],
              get_t(x, filter.b_index));
        }
        if (filter.b_index != solution_node_) {
          grad[NodeToSolutionIndex(filter.b_index)] += filter.filter->DCostDtb(
              base_clock_[i], get_t(x, i), base_clock_[filter.b_index],
              get_t(x, filter.b_index));
        }
      }
    }
  }

  double cost = 0;
  for (size_t i = 0u; i < filters_.size(); ++i) {
    for (const struct FilterPair &filter : filters_[i]) {
      cost += filter.filter->Cost(base_clock_[i], get_t(x, i),
                                  base_clock_[filter.b_index],
                                  get_t(x, filter.b_index));
    }
  }
  return cost;
}

void MultiNodeNoncausalOffsetEstimator::Start(
    SimulatedEventLoopFactory *factory) {
  for (std::pair<const std::tuple<const Node *, const Node *>,
                 message_bridge::NoncausalOffsetEstimator> &filter : filters_) {
    const Node *const node_a = std::get<0>(filter.first);
    const Node *const node_b = std::get<1>(filter.first);

    filter.second.SetFirstFwdTime(
        factory->GetNodeEventLoopFactory(node_a)->monotonic_now());
    filter.second.SetFirstRevTime(
        factory->GetNodeEventLoopFactory(node_b)->monotonic_now());
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

void MultiNodeNoncausalOffsetEstimator::LogFit(std::string_view prefix) {
  for (size_t node_index = 0; node_index < nodes_count(); ++node_index) {
    const Node *node = configuration::GetNode(configuration(), node_index);
    VLOG(1)
        << node->name()->string_view() << " now "
        << event_loop_factory_->GetNodeEventLoopFactory(node)->monotonic_now()
        << " distributed " << event_loop_factory_->distributed_now();
  }

  for (std::pair<const std::tuple<const Node *, const Node *>,
                 message_bridge::NoncausalOffsetEstimator> &filter : filters_) {
    message_bridge::NoncausalOffsetEstimator *estimator = &filter.second;

    const std::deque<
        std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
        a_timestamps = estimator->ATimestamps();
    const std::deque<
        std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
        b_timestamps = estimator->BTimestamps();

    if (a_timestamps.size() == 0 && b_timestamps.size() == 0) {
      continue;
    }

    if (VLOG_IS_ON(1)) {
      estimator->LogFit(prefix);
    }

    const Node *const node_a = std::get<0>(filter.first);
    const Node *const node_b = std::get<1>(filter.first);

    const size_t node_a_index =
        configuration::GetNodeIndex(configuration(), node_a);
    const size_t node_b_index =
        configuration::GetNodeIndex(configuration(), node_b);

    NodeEventLoopFactory *node_a_factory =
        event_loop_factory_->GetNodeEventLoopFactory(node_a);
    NodeEventLoopFactory *node_b_factory =
        event_loop_factory_->GetNodeEventLoopFactory(node_b);

    const double recovered_slope =
        slope(node_b_index) / slope(node_a_index) - 1.0;
    const int64_t recovered_offset =
        offset(node_b_index).count() - offset(node_a_index).count() *
                                           slope(node_b_index) /
                                           slope(node_a_index);

    VLOG(2) << "Recovered slope " << std::setprecision(20) << recovered_slope
            << " (error " << recovered_slope - estimator->fit().slope() << ") "
            << " offset " << std::setprecision(20) << recovered_offset
            << " (error "
            << recovered_offset - estimator->fit().offset().count() << ")";

    const aos::distributed_clock::time_point a0 =
        node_a_factory->ToDistributedClock(std::get<0>(a_timestamps[0]));
    const aos::distributed_clock::time_point a1 =
        node_a_factory->ToDistributedClock(std::get<0>(a_timestamps[1]));

    VLOG(2) << node_a->name()->string_view()
            << " timestamps()[0] = " << std::get<0>(a_timestamps[0]) << " -> "
            << a0 << " distributed -> " << node_b->name()->string_view() << " "
            << node_b_factory->FromDistributedClock(a0) << " should be "
            << aos::monotonic_clock::time_point(
                   std::chrono::nanoseconds(static_cast<int64_t>(
                       std::get<0>(a_timestamps[0]).time_since_epoch().count() *
                       (1.0 + estimator->fit().slope()))) +
                   estimator->fit().offset())
            << ((a0 <= event_loop_factory_->distributed_now())
                    ? ""
                    : " After now, investigate");
    VLOG(2) << node_a->name()->string_view()
            << " timestamps()[1] = " << std::get<0>(a_timestamps[1]) << " -> "
            << a1 << " distributed -> " << node_b->name()->string_view() << " "
            << node_b_factory->FromDistributedClock(a1) << " should be "
            << aos::monotonic_clock::time_point(
                   std::chrono::nanoseconds(static_cast<int64_t>(
                       std::get<0>(a_timestamps[1]).time_since_epoch().count() *
                       (1.0 + estimator->fit().slope()))) +
                   estimator->fit().offset())
            << ((event_loop_factory_->distributed_now() <= a1)
                    ? ""
                    : " Before now, investigate");

    const aos::distributed_clock::time_point b0 =
        node_b_factory->ToDistributedClock(std::get<0>(b_timestamps[0]));
    const aos::distributed_clock::time_point b1 =
        node_b_factory->ToDistributedClock(std::get<0>(b_timestamps[1]));

    VLOG(2) << node_b->name()->string_view()
            << " timestamps()[0] = " << std::get<0>(b_timestamps[0]) << " -> "
            << b0 << " distributed -> " << node_a->name()->string_view() << " "
            << node_a_factory->FromDistributedClock(b0)
            << ((b0 <= event_loop_factory_->distributed_now())
                    ? ""
                    : " After now, investigate");
    VLOG(2) << node_b->name()->string_view()
            << " timestamps()[1] = " << std::get<0>(b_timestamps[1]) << " -> "
            << b1 << " distributed -> " << node_a->name()->string_view() << " "
            << node_a_factory->FromDistributedClock(b1)
            << ((event_loop_factory_->distributed_now() <= b1)
                    ? ""
                    : " Before now, investigate");
  }
}

void MultiNodeNoncausalOffsetEstimator::UpdateOffsets() {
  for (size_t node_index = 0; node_index < nodes_count(); ++node_index) {
    const Node *node = configuration::GetNode(configuration(), node_index);
    VLOG(1)
        << node->name()->string_view() << " before "
        << event_loop_factory_->GetNodeEventLoopFactory(node)->monotonic_now();
  }
  VLOG(1) << "Distributed " << event_loop_factory_->distributed_now();

  std::tie(time_slope_matrix_, time_offset_matrix_) = SolveOffsets();
  Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[",
                           "]");

  for (size_t node_index = 0; node_index < nodes_count(); ++node_index) {
    const Node *node = configuration::GetNode(configuration(), node_index);
    event_loop_factory_->GetNodeEventLoopFactory(node)->SetDistributedOffset(
        offset(node_index), slope(node_index));

    VLOG(1) << "Offset for node " << node_index << " "
            << node->name()->string_view() << " is "
            << aos::distributed_clock::time_point(offset(node_index))
            << " slope " << std::setprecision(9) << std::fixed
            << slope(node_index);
  }

  for (std::pair<const std::tuple<const Node *, const Node *>,
                 message_bridge::NoncausalOffsetEstimator> &filter : filters_) {
    // TODO(austin): Do we need to freeze up until a time?  If we freeze a
    // single point line segment, we are really assuming that it will never
    // deviate from horizontal again.
    filter.second.Freeze();
  }

  for (size_t node_index = 0; node_index < nodes_count(); ++node_index) {
    const Node *node = configuration::GetNode(configuration(), node_index);
    VLOG(1)
        << node->name()->string_view() << " after "
        << event_loop_factory_->GetNodeEventLoopFactory(node)->monotonic_now();
  }
}

void MultiNodeNoncausalOffsetEstimator::Initialize(
    const Configuration *logged_configuration) {
  logged_configuration_ = logged_configuration;
  // We need to now seed our per-node time offsets and get everything set up
  // to run.
  const size_t num_nodes = nodes_count();

  // It is easiest to solve for per node offsets with a matrix rather than
  // trying to solve the equations by hand.  So let's get after it.
  //
  // Now, build up the map matrix.
  //
  // offset_matrix_ = (map_matrix_ + slope_matrix_) * [ta; tb; tc]
  map_matrix_ = Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>::Zero(
      filters_.size() + 1, num_nodes);
  slope_matrix_ =
      Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>::Zero(
          filters_.size() + 1, num_nodes);

  offset_matrix_ =
      Eigen::Matrix<mpq_class, Eigen::Dynamic, 1>::Zero(filters_.size() + 1);
  valid_matrix_ =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(filters_.size() + 1);
  last_valid_matrix_ =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(filters_.size() + 1);

  time_offset_matrix_ = Eigen::VectorXd::Zero(num_nodes);
  time_slope_matrix_ = Eigen::VectorXd::Zero(num_nodes);

  // All times should average out to the distributed clock.
  for (int i = 0; i < map_matrix_.cols(); ++i) {
    // 1/num_nodes.
    map_matrix_(0, i) = mpq_class(1, num_nodes);
  }
  valid_matrix_(0) = true;

  {
    // Now, add the a - b -> sample elements.
    size_t i = 1;
    for (std::pair<const std::tuple<const Node *, const Node *>,
                   message_bridge::NoncausalOffsetEstimator> &filter :
         filters_) {
      const Node *const node_a = std::get<0>(filter.first);
      const Node *const node_b = std::get<1>(filter.first);

      const size_t node_a_index =
          configuration::GetNodeIndex(configuration(), node_a);
      const size_t node_b_index =
          configuration::GetNodeIndex(configuration(), node_b);

      // -a
      map_matrix_(i, node_a_index) = mpq_class(-1);
      // +b
      map_matrix_(i, node_b_index) = mpq_class(1);

      // -> sample
      filter.second.set_slope_pointer(&slope_matrix_(i, node_a_index));
      filter.second.set_offset_pointer(&offset_matrix_(i, 0));

      valid_matrix_(i) = false;
      filter.second.set_valid_pointer(&valid_matrix_(i));

      ++i;
    }
  }

  const size_t connected_nodes = ConnectedNodes();

  // We don't need to support isolated nodes until someone has a real use
  // case.
  CHECK_EQ(connected_nodes, num_nodes)
      << ": There is a node which isn't communicating with the rest.";
}

std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
           Eigen::Matrix<double, Eigen::Dynamic, 1>>
MultiNodeNoncausalOffsetEstimator::SolveOffsets() {
  // TODO(austin): Split this out and unit tests a bit better.  When we do
  // partial node subsets and also try to optimize this again would be a good
  // time.
  //
  // TODO(austin): CHECK that the number doesn't change over time.  We can freak
  // out if that happens.

  // Start by counting how many node pairs we have an offset estimated for.
  int nonzero_offset_count = 1;
  for (int i = 1; i < valid_matrix_.rows(); ++i) {
    if (valid_matrix_(i) != 0) {
      ++nonzero_offset_count;
    }
  }

  Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[",
                           "]");

  // If there are missing rows, we can't solve the original problem and instead
  // need to filter the matrix to remove the missing rows and solve a simplified
  // problem.  What this means practically is that we might have pairs of nodes
  // which are communicating, but we don't have timestamps between.  But we can
  // have multiple paths in our graph between 2 nodes, so we can still solve
  // time without the missing timestamp.
  //
  // In the following example, we can drop any of the last 3 rows, and still
  // solve.
  //
  // [1/3  1/3  1/3  ] [ta]   [t_distributed]
  // [ 1  -1-m1  0   ] [tb] = [oab]
  // [ 1    0  -1-m2 ] [tc]   [oac]
  // [ 0    1  -1-m2 ]        [obc]
  if (nonzero_offset_count != offset_matrix_.rows()) {
    Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> mpq_map =
        Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>::Zero(
            nonzero_offset_count, map_matrix_.cols());
    Eigen::Matrix<mpq_class, Eigen::Dynamic, 1> mpq_offsets =
        Eigen::Matrix<mpq_class, Eigen::Dynamic, 1>::Zero(nonzero_offset_count);

    std::vector<bool> valid_nodes(nodes_count(), false);

    size_t destination_row = 0;
    for (int j = 0; j < map_matrix_.cols(); ++j) {
      mpq_map(0, j) = mpq_class(1, map_matrix_.cols());
    }
    mpq_offsets(0) = mpq_class(0);
    ++destination_row;

    for (int i = 1; i < offset_matrix_.rows(); ++i) {
      // Copy over the first row, i.e. the row which says that all times average
      // to the distributed time.  And then copy over all valid rows.
      if (valid_matrix_(i)) {
        mpq_offsets(destination_row) = mpq_class(offset_matrix_(i));

        for (int j = 0; j < map_matrix_.cols(); ++j) {
          mpq_map(destination_row, j) = map_matrix_(i, j) + slope_matrix_(i, j);
          if (mpq_map(destination_row, j) != 0) {
            valid_nodes[j] = true;
          }
        }

        ++destination_row;
      }
    }

    VLOG(1) << "Filtered map " << ToDouble(mpq_map).format(HeavyFmt);
    VLOG(1) << "Filtered offsets " << ToDouble(mpq_offsets).format(HeavyFmt);

    // Compute (and cache) the current connectivity.  If we have N nodes
    // configured, but logs only from one of them, we want to assume that the
    // rest of the nodes match the distributed clock exactly.
    //
    // If data shows up later for them, we will CHECK when time jumps.
    //
    // TODO(austin): Once we have more info on what cases are reasonable, we can
    // open up the restrictions.
    if (valid_matrix_ != last_valid_matrix_) {
      Eigen::FullPivLU<Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>>
          full_piv(mpq_map);
      const size_t connected_nodes = full_piv.rank();

      size_t valid_node_count = 0;
      for (size_t i = 0; i < valid_nodes.size(); ++i) {
        const bool valid_node = valid_nodes[i];
        if (valid_node) {
          ++valid_node_count;
        } else {
          LOG(WARNING)
              << "Node "
              << logged_configuration()->nodes()->Get(i)->name()->string_view()
              << " has no observations, setting to distributed clock.";
        }
      }

      // Confirm that the set of nodes we have connected matches the rank.
      // Otherwise a<->b and c<->d would count as 4 but really is 3.
      CHECK_EQ(std::max(static_cast<size_t>(1u), valid_node_count),
               connected_nodes)
          << ": Ambiguous nodes.";

      last_valid_matrix_ = valid_matrix_;
      cached_valid_node_count_ = valid_node_count;
    }

    // There are 2 cases.  Either all the nodes are connected with each other by
    // actual data, or we have isolated nodes.  We want to force the isolated
    // nodes to match the distributed clock exactly, and to solve for the other
    // nodes.
    if (cached_valid_node_count_ == 0) {
      // Cheat.  If there are no valid nodes, the slopes are 1, and offset is 0,
      // ie, just be the distributed clock.
      return std::make_tuple(
          Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(nodes_count()),
          Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(nodes_count()));
    } else if (cached_valid_node_count_ == nodes_count()) {
      return Solve(mpq_map, mpq_offsets);
    } else {
      // Strip out any columns (nodes) which aren't relevant.  Solve the
      // simplified problem, then set any nodes which were missing back to slope
      // 1, offset 0 (ie the distributed clock).
      Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>
          valid_node_mpq_map =
              Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>::Zero(
                  nonzero_offset_count, cached_valid_node_count_);

      {
        // Only copy over the columns with valid nodes in them.
        size_t column = 0;
        for (size_t i = 0; i < valid_nodes.size(); ++i) {
          if (valid_nodes[i]) {
            valid_node_mpq_map.col(column) = mpq_map.col(i);

            ++column;
          }
        }
        // The 1/n needs to be based on the number of nodes being solved.
        // Recreate it here.
        for (int j = 0; j < valid_node_mpq_map.cols(); ++j) {
          valid_node_mpq_map(0, j) = mpq_class(1, cached_valid_node_count_);
        }
      }

      VLOG(1) << "Reduced node filtered map "
              << ToDouble(valid_node_mpq_map).format(HeavyFmt);
      VLOG(1) << "Reduced node filtered offsets "
              << ToDouble(mpq_offsets).format(HeavyFmt);

      // Solve the simplified problem now.
      std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
                 Eigen::Matrix<double, Eigen::Dynamic, 1>>
          valid_result = Solve(valid_node_mpq_map, mpq_offsets);

      // And expand the results back into a solution matrix.
      std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
                 Eigen::Matrix<double, Eigen::Dynamic, 1>>
          result = std::make_tuple(
              Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(nodes_count()),
              Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(nodes_count()));

      {
        size_t column = 0;
        for (size_t i = 0; i < valid_nodes.size(); ++i) {
          if (valid_nodes[i]) {
            std::get<0>(result)(i) = std::get<0>(valid_result)(column);
            std::get<1>(result)(i) = std::get<1>(valid_result)(column);

            ++column;
          }
        }
      }

      return result;
    }
  } else {
    const Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> mpq_map =
        map_matrix_ + slope_matrix_;
    VLOG(2) << "map " << ToDouble(map_matrix_ + slope_matrix_).format(HeavyFmt);
    VLOG(2) << "offsets " << ToDouble(offset_matrix_).format(HeavyFmt);

    return Solve(mpq_map, offset_matrix_);
  }
}

size_t MultiNodeNoncausalOffsetEstimator::ConnectedNodes() {
  // Rank of the map matrix tells you if all the nodes are in communication
  // with each other, which tells you if the offsets are observable.
  return Eigen::FullPivLU<
             Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>>(
             map_matrix_)
      .rank();
}

}  // namespace message_bridge
}  // namespace aos
