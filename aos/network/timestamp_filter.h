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

// Max velocity to clamp the filter to in seconds/second.
inline constexpr double kMaxVelocity() { return 0.001; }

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
  // Forces the offset and time to the provided sample without filtering.  Used
  // for syncing with a remote filter calculation.
  void Set(aos::monotonic_clock::time_point monotonic_now,
           std::chrono::nanoseconds sample_ns);

  double velocity_contribution() const { return velocity_contribution_; }
  double sample_contribution() const { return sample_contribution_; }
  double time_contribution() const { return time_contribution_; }
  double clamp() const { return clamp_; }

  // Updates with a new sample.  monotonic_now is the timestamp of the sample on
  // the destination node, and sample_ns is destination_time - source_time.
  void Sample(aos::monotonic_clock::time_point monotonic_now,
              std::chrono::nanoseconds sample_ns);

  // Updates the base_offset, and compensates offset while we are here.
  void set_base_offset(std::chrono::nanoseconds base_offset);

  double offset() const {
    VLOG(1) << " " << this << " offset " << offset_;
    return offset_;
  }

  std::chrono::nanoseconds base_offset() const { return base_offset_; }

  double base_offset_double() const {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
               base_offset_)
        .count();
  }

  bool has_sample() const {
    return last_time_ != aos::monotonic_clock::min_time;
  }

  double velocity() const { return state_velocity_; }
  double filtered_velocity() const { return filtered_velocity_; }
  double last_velocity_sample() const {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
               last_velocity_sample_ns_)
        .count();
  }

  aos::monotonic_clock::time_point last_time() const { return last_time_; }

  void Reset();

 private:
  void VelocitySample(aos::monotonic_clock::time_point monotonic_now,
                      std::chrono::nanoseconds sample_ns);

  double offset_ = 0;
  double state_velocity_ = 0.0;
  double velocity_ = 0;
  double filtered_velocity_ = 0;
  double filtered_velocity_time_ = 0;

  double velocity_contribution_ = 0.0;
  double clamp_ = 0.0;
  double sample_contribution_ = 0.0;
  double time_contribution_ = 0.0;

  std::chrono::nanoseconds last_sample_ns_;

  aos::monotonic_clock::time_point last_time_ = aos::monotonic_clock::min_time;
  std::chrono::nanoseconds base_offset_{0};

  aos::monotonic_clock::time_point last_velocity_sample_time_ =
      aos::monotonic_clock::min_time;
  std::chrono::nanoseconds last_velocity_sample_ns_{0};
};

// This class combines the a -> b offsets with the b -> a offsets and
// aggressively filters the results.
class ClippedAverageFilter {
 public:
  ~ClippedAverageFilter() {
    if (fwd_fp_ != nullptr) {
      fclose(fwd_fp_);
    }
    if (rev_fp_ != nullptr) {
      fclose(rev_fp_);
    }
  }

  // Sets the forward sample without filtering.  See FwdSample for more details.
  void FwdSet(aos::monotonic_clock::time_point monotonic_now,
              std::chrono::nanoseconds sample_ns);

  // Adds a forward sample.  sample_ns = destination - source;  Forward samples
  // are from A -> B.
  void FwdSample(aos::monotonic_clock::time_point monotonic_now,
                 std::chrono::nanoseconds sample_ns);

  // Sets the forward sample without filtering.  See FwdSample for more details.
  void RevSet(aos::monotonic_clock::time_point monotonic_now,
              std::chrono::nanoseconds sample_ns);

  // Adds a reverse sample.  sample_ns = destination - source;  Reverse samples
  // are B -> A.
  void RevSample(aos::monotonic_clock::time_point monotonic_now,
                 std::chrono::nanoseconds sample_ns);

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
  void set_base_offset(std::chrono::nanoseconds base_offset);

  bool MissingSamples() {
    return (last_fwd_time_ == aos::monotonic_clock::min_time) ||
           (last_rev_time_ == aos::monotonic_clock::min_time);
  }

  void Reset();

  aos::monotonic_clock::time_point fwd_last_time() const {
    return fwd_.last_time();
  }
  aos::monotonic_clock::time_point rev_last_time() const {
    return rev_.last_time();
  }

  // If not nullptr, timestamps will get written to these two files for
  // debugging.
  void SetFwdCsvFileName(std::string_view name);
  void SetRevCsvFileName(std::string_view name);

  void set_first_fwd_time(aos::monotonic_clock::time_point time);
  void set_first_rev_time(aos::monotonic_clock::time_point time);

 private:
  // Updates the offset estimate given the current time, and a pointer to the
  // variable holding the last time.
  void Update(aos::monotonic_clock::time_point monotonic_now,
              aos::monotonic_clock::time_point *last_time);

  // Filters for both the forward and reverse directions.
  TimestampFilter fwd_;
  TimestampFilter rev_;

  // Base offset in nanoseconds.  This is subtracted from all calculations, and
  // added back to the result when reporting.
  std::chrono::nanoseconds base_offset_ = std::chrono::nanoseconds(0);
  // Dynamic part of the offset.
  double offset_ = 0;
  double offset_velocity_ = 0;

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

  std::string rev_csv_file_name_;
  std::string fwd_csv_file_name_;
  FILE *fwd_fp_ = nullptr;
  FILE *rev_fp_ = nullptr;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
