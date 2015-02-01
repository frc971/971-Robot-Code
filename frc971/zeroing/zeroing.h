#ifndef FRC971_ZEROING_ZEROING_H_
#define FRC971_ZEROING_ZEROING_H_

#include <vector>
#include "frc971/zeroing/zeroing_queue.q.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with encoder,
// the pot and the indices.
class ZeroingEstimator {
 public:
  ZeroingEstimator(double index_difference, size_t max_sample_count);
  void UpdateEstimate(const ZeroingInfo& info);
  double getPosition();
 private:
  // The estimated position.
  double pos_;
  // The distance between two consecutive index positions.
  double index_diff_;
  // The next position in 'start_pos_samples_' to be used to store the
  // next sample.
  int samples_idx_;
  // Last 'max_sample_count_' samples for start positions.
  std::vector<double> start_pos_samples_;
  // The number of the last samples of start position to consider
  // in the estimation.
  size_t max_sample_count_;
};

} // namespace zeroing
} // namespace frc971

#endif  // FRC971_ZEROING_ZEROING_H_
