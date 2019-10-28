#include "frc971/wpilib/dma_edge_counting.h"

#include "aos/logging/logging.h"

namespace frc971 {
namespace wpilib {

bool DMAEdgeCounter::ExtractValue(const DMASample &sample) const {
  return sample.Get(input_);
}

void DMAEdgeCounter::UpdateFromSample(const DMASample &sample) {
  const bool previous_value =
      have_prev_sample_ ? ExtractValue(prev_sample_) : previous_polled_value_;
  have_prev_sample_ = true;
  prev_sample_ = sample;

  if (!previous_value && ExtractValue(sample)) {
    pos_edge_count_++;
    pos_last_encoder_ = sample.GetRaw(encoder_);
  } else if (previous_value && !ExtractValue(sample)) {
    neg_edge_count_++;
    neg_last_encoder_ = sample.GetRaw(encoder_);
  }
}

void DMAPulseWidthReader::UpdateFromSample(const DMASample &sample) {
  if (have_prev_sample_ && prev_sample_.Get(input_) && !sample.Get(input_)) {
    last_width_ = sample.GetTimestamp() - prev_sample_.GetTimestamp();
  }
  have_prev_sample_ = true;
  prev_sample_ = sample;
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
          if (sample_time_ < static_cast<int64_t>(current_sample.GetTime())) {
            // If the latest DMA sample happened after we started polling, then
            // just use the values from it because they're more recent.
            for (auto &c : handlers_) {
              c->PollFromSample(current_sample);
            }
          }
          return;
        }
        break;
      case DMA::STATUS_TIMEOUT:
        return;
      case DMA::STATUS_ERROR:
        AOS_LOG(WARNING, "DMA read failed\n");
        break;
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
