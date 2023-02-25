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
  if (have_prev_sample_ && high_time_ != 0 && prev_sample_.Get(input_) &&
      !sample.Get(input_)) {
    last_width_ = (sample.GetTime() - high_time_) * 1e-6;
    high_time_ = 0;
    poll_count_ = 0;
  } else if (have_prev_sample_ && !prev_sample_.Get(input_) &&
             sample.Get(input_)) {
    high_time_ = prev_sample_.GetTime();
    poll_count_ = 0;
  }
  have_prev_sample_ = true;
  prev_sample_ = sample;
}

void DMAPulseWidthReader::UpdatePolledValue() {
  // If we are polled without an update for too long, reset
  constexpr size_t kMaxPollCount = 4;
  if (poll_count_ > kMaxPollCount) {
    high_time_ = 0;
    have_prev_sample_ = false;
    last_width_ = ::std::numeric_limits<double>::quiet_NaN();
  }
  poll_count_++;
}

void DMAPulseSeparationReader::UpdateFromSample(const DMASample &sample) {
  // save the time of the falling edge of the input one
  if (have_prev_sample_ && !sample.Get(input_one_) &&
      prev_sample_.Get(input_one_)) {
    input_one_time_ = sample.GetTimestamp();
  }

  // take the difference in time between the falling edge of the input one and
  // the falling edge of the input two
  if (!sample.Get(input_two_) && input_one_time_.has_value()) {
    last_width_ = sample.GetTimestamp() - input_one_time_.value();
    pulses_detected_++;
    input_one_time_.reset();
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
