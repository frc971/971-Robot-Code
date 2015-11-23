#include "frc971/wpilib/dma_edge_counting.h"

#include "aos/common/logging/logging.h"

namespace frc971 {
namespace wpilib {

bool DMAEdgeCounter::ExtractValue(const DMASample &sample) const {
  return sample.Get(input_);
}

void DMAEdgeCounter::UpdateFromSample(const DMASample &sample) {
  const bool previous_value =
      have_prev_sample_ ? ExtractValue(prev_sample_) : polled_value_;
  have_prev_sample_ = true;
  prev_sample_ = sample;

  if (!previous_value && ExtractValue(sample)) {
    pos_edge_count_++;
    pos_edge_time_ = sample.GetTimestamp();
    pos_last_encoder_ = sample.GetRaw(encoder_);
  } else if (previous_value && !ExtractValue(sample)) {
    neg_edge_count_++;
    neg_edge_time_ = sample.GetTimestamp();
    neg_last_encoder_ = sample.GetRaw(encoder_);
  }
}

void DMASynchronizer::CheckDMA() {
  DMASample current_sample;

  size_t remaining = 0;
  while (true) {
    switch (dma_->Read(&current_sample, 0, &remaining)) {
      case DMA::STATUS_OK:
        for (auto &c : handlers_) {
          c->UpdateFromSample(current_sample);
        }

        if (remaining == 0) {
          if (sample_time_ < current_sample.GetTime()) {
            // If the latest DMA sample happened after we started polling, then
            // just use the values from it because they're more recent.
            for (auto &c : handlers_) {
              c->PollFromSample(current_sample);
            }
          }
          return;
        }
      case DMA::STATUS_TIMEOUT:
        return;
      case DMA::STATUS_ERROR:
        LOG(WARNING, "DMA read failed\n");
        break;
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
