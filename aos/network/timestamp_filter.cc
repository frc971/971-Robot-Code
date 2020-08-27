#include "aos/network/timestamp_filter.h"

#include <chrono>
#include <tuple>

#include "aos/time/time.h"

namespace aos {
namespace message_bridge {
namespace chrono = std::chrono;

void TimestampFilter::Set(aos::monotonic_clock::time_point monotonic_now,
                          chrono::nanoseconds sample_ns) {
  const double sample =
      chrono::duration_cast<chrono::duration<double>>(sample_ns - base_offset_)
          .count();
  offset_ = sample;
  last_time_ = monotonic_now;
}

void TimestampFilter::Sample(aos::monotonic_clock::time_point monotonic_now,
                             chrono::nanoseconds sample_ns) {
  // Compute the sample offset as a double (seconds), taking into account the
  // base offset.
  const double sample =
      chrono::duration_cast<chrono::duration<double>>(sample_ns - base_offset_)
          .count();

  // This is our first sample.  Just use it.
  if (last_time_ == aos::monotonic_clock::min_time) {
    offset_ = sample;
  } else {
    // Took less time to transmit, so clamp to it.
    if (sample < offset_) {
      offset_ = sample;
    } else {
      // We split things up into 2 portions.
      //  1) Each sample has information.  Correct some using it.
      //  2) We want to keep a decent time constant if the sample rate slows.
      //     Take time since the last sample into account.

      // Time constant for the low pass filter in seconds.
      constexpr double kTau = 0.5;

      constexpr double kClampNegative = -0.0003;

      {
        // 1)
        constexpr double kAlpha = 0.005;
        // This formulation is more numerically precise.
        // Clamp to kClampNegative ms to reduce the effect of wildly large
        // samples.
        offset_ = offset_ - kAlpha * std::max(offset_ - sample, kClampNegative);
      }

      {
        // 2)
        //
        // 1-e^(t/tau) -> alpha
        const double alpha =
            -std::expm1(-chrono::duration_cast<chrono::duration<double>>(
                             monotonic_now - last_time_)
                             .count() /
                        kTau);

        // Clamp to kClampNegative ms to reduce the effect of wildly large
        // samples.
        offset_ = offset_ - alpha * std::max(offset_ - sample, kClampNegative);
      }
    }
  }

  last_time_ = monotonic_now;
}

void TimestampFilter::set_base_offset(chrono::nanoseconds base_offset) {
  offset_ -= chrono::duration_cast<chrono::duration<double>>(base_offset -
                                                             base_offset_)
                 .count();
  base_offset_ = base_offset;
  // Clear everything out to avoid any numerical precision problems.
  last_time_ = aos::monotonic_clock::min_time;
}

void TimestampFilter::Reset() {
  offset_ = 0;

  last_time_ = aos::monotonic_clock::min_time;
  base_offset_ = chrono::nanoseconds(0);
}

void ClippedAverageFilter::FwdSet(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  fwd_.Set(monotonic_now, sample_ns);
  Update(monotonic_now, &last_fwd_time_);
}

void ClippedAverageFilter::FwdSample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  fwd_.Sample(monotonic_now, sample_ns);
  Update(monotonic_now, &last_fwd_time_);

  if (fwd_fp != nullptr) {
    if (first_fwd_time_ == aos::monotonic_clock::min_time) {
      first_fwd_time_ = monotonic_now;
    }
    fprintf(fwd_fp, "%f, %f, %f, %f\n",
            chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                            first_fwd_time_)
                .count(),
            chrono::duration_cast<chrono::duration<double>>(sample_ns).count(),
            fwd_.offset() + fwd_.base_offset_double(),
            chrono::duration_cast<chrono::duration<double>>(offset()).count());
  }
}

void ClippedAverageFilter::RevSet(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  rev_.Set(monotonic_now, sample_ns);
  Update(monotonic_now, &last_rev_time_);
}

void ClippedAverageFilter::RevSample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  rev_.Sample(monotonic_now, sample_ns);
  Update(monotonic_now, &last_rev_time_);

  if (rev_fp != nullptr) {
    if (first_rev_time_ == aos::monotonic_clock::min_time) {
      first_rev_time_ = monotonic_now;
    }
    fprintf(rev_fp, "%f, %f, %f, %f\n",
            chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                            first_rev_time_)
                .count(),
            chrono::duration_cast<chrono::duration<double>>(sample_ns).count(),
            rev_.offset() + rev_.base_offset_double(),
            chrono::duration_cast<chrono::duration<double>>(offset()).count());
  }
}

void ClippedAverageFilter::set_base_offset(chrono::nanoseconds base_offset) {
  offset_ -= chrono::duration_cast<chrono::duration<double>>(base_offset -
                                                             base_offset_)
                 .count();
  fwd_.set_base_offset(base_offset);
  rev_.set_base_offset(-base_offset);
  base_offset_ = base_offset;
  last_fwd_time_ = aos::monotonic_clock::min_time;
  last_rev_time_ = aos::monotonic_clock::min_time;
}

void ClippedAverageFilter::Reset() {
  base_offset_ = chrono::nanoseconds(0);
  offset_ = 0;

  last_fwd_time_ = aos::monotonic_clock::min_time;
  last_rev_time_ = aos::monotonic_clock::min_time;
  first_fwd_time_ = aos::monotonic_clock::min_time;
  first_rev_time_ = aos::monotonic_clock::min_time;

  fwd_.Reset();
  rev_.Reset();
}

void ClippedAverageFilter::Update(
    aos::monotonic_clock::time_point monotonic_now,
    aos::monotonic_clock::time_point *last_time) {
  // ta = t + offseta
  // tb = t + offsetb
  // fwd sample => ta - tb + network -> offseta - offsetb + network
  // rev sample => tb - ta + network -> offsetb - offseta + network
  const double hard_max = fwd_.offset();
  const double hard_min = -rev_.offset();
  const double average = (hard_max + hard_min) / 2.0;
  // We don't want to clip the offset to the hard min/max.  We really want to
  // keep it within a band around the middle.  ratio of 0.5 means stay within
  // +- 0.25 of the middle of the hard min and max.
  constexpr double kBand = 0.5;
  const double max = average + kBand / 2.0 * (hard_max - hard_min);
  const double min = average - kBand / 2.0 * (hard_max - hard_min);

  // Update regardless for the first sample from both the min and max.
  if (*last_time == aos::monotonic_clock::min_time) {
    offset_ = average;
  } else {
    // Do just a time constant based update.  We can afford to be slow here
    // for smoothness.
    constexpr double kTau = 10.0;
    const double alpha =
        -std::expm1(-chrono::duration_cast<chrono::duration<double>>(
                         monotonic_now - *last_time)
                         .count() /
                    kTau);

    // Clamp it such that it remains in the min/max bounds.
    offset_ = std::clamp(offset_ - alpha * (offset_ - average), min, max);
  }
  *last_time = monotonic_now;

  if (sample_pointer_ != nullptr) {
    // TODO(austin): Probably shouldn't do the update if we don't have fwd and
    // reverse samples.
    if (!MissingSamples()) {
      *sample_pointer_ = offset_;
      VLOG(1) << "Updating sample to " << offset_;
    } else {
      LOG(WARNING) << "Don't have both samples.";
    }
  }
}

}  // namespace message_bridge
}  // namespace aos
