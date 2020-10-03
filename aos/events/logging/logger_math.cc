#include "aos/events/logging/logger.h"

#include "Eigen/Dense"

#include "third_party/gmp/gmpxx.h"

namespace aos {
namespace logger {

namespace {
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

  VLOG(1) << "Took "
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

// This is slow to compile, so we put it in a separate file.  More parallelism
// and less change.
std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
           Eigen::Matrix<double, Eigen::Dynamic, 1>>
LogReader::SolveOffsets() {
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
    VLOG(1) << "map " << (map_matrix_ + slope_matrix_).format(HeavyFmt);
    VLOG(1) << "offsets " << offset_matrix_.format(HeavyFmt);

    return Solve(mpq_map, offset_matrix_);
  }
}

}  // namespace logger
}  // namespace aos
