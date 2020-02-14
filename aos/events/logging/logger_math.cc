#include "aos/events/logging/logger.h"

#include "Eigen/Dense"

namespace aos {
namespace logger {

// This is slow to compile, so we put it in a separate file.  More parallelism
// and less change.
Eigen::Matrix<double, Eigen::Dynamic, 1> LogReader::SolveOffsets() {
  return map_matrix_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
      .solve(sample_matrix_);
}

}  // namespace logger
}  // namespace aos
