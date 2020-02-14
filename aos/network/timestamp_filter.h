#ifndef AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
#define AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_

#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "aos/time/time.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

// This class handles filtering differences between clocks across a network.
//
// The basic concept is that network latencies are highly asymmetric.  They will
// have a good and well-defined minima, and have large maxima.  We also want
// to guarantee that the filtered line is *always* below the samples.
//
// In order to preserve precision, an initial offset (base_offset_) is
// subtracted from everything.  This should make all the offsets near 0, so we
// have enough precision to represent nanoseconds, and to not have to worry too
// much about precision when solving for the global offset.
class TimestampFilter {
 public:
  // Updates with a new sample.  monotonic_now is the timestamp of the sample on
  // the destination node, and sample_ns is destination_time - source_time.
  void Sample(aos::monotonic_clock::time_point monotonic_now,
              std::chrono::nanoseconds sample_ns) {
    // Compute the sample offset as a double (seconds), taking into account the
    // base offset.
    const double sample =
        std::chrono::duration_cast<std::chrono::duration<double>>(sample_ns -
                                                                  base_offset_)
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
          offset_ =
              offset_ - kAlpha * std::max(offset_ - sample, kClampNegative);
        }

        {
          // 2)
          //
          // 1-e^(t/tau) -> alpha
          const double alpha = -std::expm1(
              -std::chrono::duration_cast<std::chrono::duration<double>>(
                   monotonic_now - last_time_)
                   .count() /
              kTau);

          // Clamp to kClampNegative ms to reduce the effect of wildly large
          // samples.
          offset_ =
              offset_ - alpha * std::max(offset_ - sample, kClampNegative);
        }
      }
    }

    last_time_ = monotonic_now;
  }

  // Updates the base_offset, and compensates offset while we are here.
  void set_base_offset(std::chrono::nanoseconds base_offset) {
    offset_ -= std::chrono::duration_cast<std::chrono::duration<double>>(
                   base_offset - base_offset_)
                   .count();
    base_offset_ = base_offset;
    // Clear everything out to avoid any numerical precision problems.
    last_time_ = aos::monotonic_clock::min_time;
  }

  double offset() const { return offset_; }

  std::chrono::nanoseconds base_offset() const { return base_offset_; }

  double base_offset_double() const {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
               base_offset_)
        .count();
  }

  bool has_sample() const {
    return last_time_ != aos::monotonic_clock::min_time;
  }

 private:
  double offset_ = 0;

  aos::monotonic_clock::time_point last_time_ = aos::monotonic_clock::min_time;
  std::chrono::nanoseconds base_offset_{0};
};

// This class combines the a -> b offsets with the b -> a offsets and
// aggressively filters the results.
struct ClippedAverageFilter {
  // If not nullptr, timestamps will get written to these two files for
  // debugging.
  FILE *fwd_fp = nullptr;
  FILE *rev_fp = nullptr;

  ~ClippedAverageFilter() {
    if (fwd_fp != nullptr) {
      fclose(fwd_fp);
    }
    if (rev_fp != nullptr) {
      fclose(rev_fp);
    }
  }

  // Adds a forward sample.  sample_ns = destination - source;  Forward samples
  // are from A -> B.
  void FwdSample(aos::monotonic_clock::time_point monotonic_now,
                 std::chrono::nanoseconds sample_ns) {
    fwd_.Sample(monotonic_now, sample_ns);
    Update(monotonic_now, &last_fwd_time_);

    if (fwd_fp != nullptr) {
      if (first_fwd_time_ == aos::monotonic_clock::min_time) {
        first_fwd_time_ = monotonic_now;
      }
      fprintf(
          fwd_fp, "%f, %f, %f, %f\n",
          std::chrono::duration_cast<std::chrono::duration<double>>(
              monotonic_now - first_fwd_time_)
              .count(),
          std::chrono::duration_cast<std::chrono::duration<double>>(sample_ns)
              .count(),
          fwd_.offset() + fwd_.base_offset_double(),
          std::chrono::duration_cast<std::chrono::duration<double>>(offset())
              .count());
    }
  }

  // Adds a reverse sample.  sample_ns = destination - source;  Reverse samples
  // are B -> A.
  void RevSample(aos::monotonic_clock::time_point monotonic_now,
                 std::chrono::nanoseconds sample_ns) {
    rev_.Sample(monotonic_now, sample_ns);
    Update(monotonic_now, &last_rev_time_);

    if (rev_fp != nullptr) {
      if (first_rev_time_ == aos::monotonic_clock::min_time) {
        first_rev_time_ = monotonic_now;
      }
      fprintf(
          rev_fp, "%f, %f, %f, %f\n",
          std::chrono::duration_cast<std::chrono::duration<double>>(
              monotonic_now - first_rev_time_)
              .count(),
          std::chrono::duration_cast<std::chrono::duration<double>>(sample_ns)
              .count(),
          rev_.offset() + rev_.base_offset_double(),
          std::chrono::duration_cast<std::chrono::duration<double>>(offset())
              .count());
    }
  }

  // Returns the overall filtered offset, offseta - offsetb.
  std::chrono::nanoseconds offset() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::duration<double>(offset_)) +
           base_offset_;
  }

  // Returns the offset as a double.
  double offset_double() const {
    return std::chrono::duration_cast<std::chrono::duration<double>>(offset())
        .count();
  }

  // Sets the sample pointer.  This address gets set every time the sample
  // changes.  Makes it really easy to place in a matrix and solve.
  void set_sample_pointer(double *sample_pointer) {
    sample_pointer_ = sample_pointer;
  }

  double *sample_pointer() { return sample_pointer_; }

  // Sets the base offset.  This is used to reduce the dynamic range needed from
  // the double to something manageable.  It is subtracted from offset_.
  void set_base_offset(std::chrono::nanoseconds base_offset) {
    offset_ -= std::chrono::duration_cast<std::chrono::duration<double>>(
                   base_offset - base_offset_)
                   .count();
    fwd_.set_base_offset(base_offset);
    rev_.set_base_offset(-base_offset);
    base_offset_ = base_offset;
    last_fwd_time_ = aos::monotonic_clock::min_time;
    last_rev_time_ = aos::monotonic_clock::min_time;
  }

 private:
  // Updates the offset estimate given the current time, and a pointer to the
  // variable holding the last time.
  void Update(aos::monotonic_clock::time_point monotonic_now,
              aos::monotonic_clock::time_point *last_time) {
    // ta = t + offseta
    // tb = t + offsetb
    // fwd sample => ta - tb + network -> offseta - offsetb + network
    // rev sample => tb - ta + network -> offsetb - offseta + network
    const double hard_max = fwd_.offset();
    const double hard_min = -rev_.offset();
    const double average = (hard_max + hard_min) / 2.0;
    LOG(INFO) << "max " << hard_max << " min " << hard_min;
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
      const double alpha = -std::expm1(
          -std::chrono::duration_cast<std::chrono::duration<double>>(
               monotonic_now - *last_time)
               .count() /
          kTau);

      // Clamp it such that it remains in the min/max bounds.
      offset_ = std::clamp(offset_ - alpha * (offset_ - average), min, max);
    }
    *last_time = monotonic_now;

    if (sample_pointer_ != nullptr) {
      *sample_pointer_ = offset_;
    }
  }

  // Filters for both the forward and reverse directions.
  TimestampFilter fwd_;
  TimestampFilter rev_;

  // Base offset in nanoseconds.  This is subtracted from all calculations, and
  // added back to the result when reporting.
  std::chrono::nanoseconds base_offset_ = std::chrono::nanoseconds(0);
  // Dynamic part of the offset.
  double offset_ = 0;

  // Last time we had a sample for a direction.
  aos::monotonic_clock::time_point last_fwd_time_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point last_rev_time_ =
      aos::monotonic_clock::min_time;

  // First times used for plotting.
  aos::monotonic_clock::time_point first_fwd_time_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point first_rev_time_ =
      aos::monotonic_clock::min_time;

  // Pointer to copy the sample to when it is updated.
  double *sample_pointer_ = nullptr;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
