#ifndef FRC971_ZEROING_ZEROING_H_
#define FRC971_ZEROING_ZEROING_H_

#include <vector>
#include "frc971/zeroing/zeroing_queue.q.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/constants.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with encoder,
// the pot and the indices.
class ZeroingEstimator {
 public:
  ZeroingEstimator(double index_difference, size_t max_sample_count);
  ZeroingEstimator(const constants::Values::ZeroingConstants &constants);
  void UpdateEstimate(const PotAndIndexPosition &info);
  void UpdateEstimate(const ZeroingInfo &info);

  double offset() const { return offset_; }
  bool zeroed() const { return zeroed_; }
  double offset_ratio_ready() const {
    return start_pos_samples_.size() / static_cast<double>(max_sample_count_);
  }

 private:
  void DoInit(double index_difference, size_t max_sample_count);

  double offset_ = 0.0;
  bool zeroed_ = false;
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
