#ifndef AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
#define AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_

#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>

#include "aos/configuration.h"
#include "aos/time/time.h"
#include "glog/logging.h"
#include "third_party/gmp/gmpxx.h"

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

// Converts a int64_t into a mpq_class.  This only uses 32 bit precision
// internally, so it will work on ARM.  This should only be used on 64 bit
// platforms to test out the 32 bit implementation.
inline mpq_class FromInt64(int64_t i) {
  uint64_t absi = std::abs(i);
  mpq_class bits(static_cast<uint32_t>((absi >> 32) & 0xffffffffu));
  bits *= mpq_class(0x10000);
  bits *= mpq_class(0x10000);
  bits += mpq_class(static_cast<uint32_t>(absi & 0xffffffffu));

  if (i < 0) {
    return -bits;
  } else {
    return bits;
  }
}

// Class to hold an affine function for the time offset.
// O(t) = slope * t + offset
//
// This is stored using mpq_class, which stores everything as full rational
// fractions.
class Line {
 public:
  Line() {}

  // Constructs a line given the offset and slope.
  Line(mpq_class offset, mpq_class slope) : offset_(offset), slope_(slope) {}

  // TODO(austin): Remove this one.
  Line(std::chrono::nanoseconds offset, double slope)
      : offset_(DoFromInt64(offset.count())), slope_(slope) {}

  // Fits a line to 2 points and returns the associated line.
  static Line Fit(
      const std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> a,
      const std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>
          b);

  // Returns the full precision slopes and offsets.
  mpq_class mpq_offset() const { return offset_; }
  mpq_class mpq_slope() const { return slope_; }

  // Returns the rounded offsets and slopes.
  std::chrono::nanoseconds offset() const {
    double o = offset_.get_d();
    return std::chrono::nanoseconds(static_cast<int64_t>(o));
  }
  double slope() const { return slope_.get_d(); }

  void Debug() const {
    LOG(INFO) << "Offset " << mpq_offset() << " slope " << mpq_slope();
  }

  // Returns the offset at a given time.
  // TODO(austin): get_d() ie double -> int64 can't be accurate...
  std::chrono::nanoseconds Eval(monotonic_clock::time_point pt) const {
    mpq_class result =
        mpq_class(FromInt64(pt.time_since_epoch().count())) * slope_ + offset_;
    return std::chrono::nanoseconds(static_cast<int64_t>(result.get_d()));
  }

 private:
  static mpq_class DoFromInt64(int64_t i) {
#if GMP_NUMB_BITS == 32
    return message_bridge::FromInt64(i);
#else
    return i;
#endif
  }

  mpq_class offset_;
  mpq_class slope_;
};

// Averages 2 fits per the equations below
//
// Oa(ta) = fa.slope * ta + fa.offset;
// tb = Oa(ta) + ta;
// Ob(tb) = fb.slope * tb + fb.offset;
// ta = Ob(tb) + tb;
//
// This splits the difference between Oa and Ob and solves:
// tb - ta = (Oa(ta) - Ob(tb)) / 2.0
// and returns O(ta) such that
// tb = O(ta) + ta
Line AverageFits(Line fa, Line fb);

// This class implements a noncausal timestamp filter.  It tracks the maximum
// points while enforcing both a maximum positive and negative slope constraint.
// It does this by building up a buffer of samples, and removing any samples
// which would create segments where the slope is invalid.  As long as the
// filter is seeded with enough future samples, the start won't change.
//
// We want the offset to be defined as tb = O(ta) + ta.  For this to work, the
// offset is (tb - ta).  If we assume tb = O(ta) + ta + network_delay, then
// O(ta) = tb - ta - network_delay.  This means that the fastest time a message
// can be delivered is going to be when the offset is the most positive.
//
// TODO(austin): Figure out how to track when we have used something from this
// filter, and when we do that, enforce that it can't change anymore.  That will
// help us find when we haven't buffered far enough in the future.
class NoncausalTimestampFilter {
 public:
  ~NoncausalTimestampFilter();

  // Returns a line fit to the oldest 2 points in the timestamp list if
  // available, or the only point (assuming 0 slope) if not available.
  Line FitLine();

  // Adds a new sample to our filtered timestamp list.
  // Returns true if adding the sample changed the output from FitLine().
  bool Sample(aos::monotonic_clock::time_point monotonic_now,
              std::chrono::nanoseconds sample_ns);

  // Removes any old timestamps from our timestamps list.
  // Returns true if adding the sample changed the output from FitLine().
  bool Pop(aos::monotonic_clock::time_point time);

  // Returns the current list of timestamps in our list.
  const std::deque<
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
      &timestamps() {
    return timestamps_;
  }

  void Debug() {
    for (std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
             timestamp : timestamps_) {
      LOG(INFO) << std::get<0>(timestamp) << " offset "
                << std::get<1>(timestamp).count();
    }
  }

  // Sets the starting point and filename to log samples to.  These functions
  // are only used when doing CSV file logging to debug the filter.
  void SetFirstTime(aos::monotonic_clock::time_point time);
  void SetCsvFileName(std::string_view name);

 private:
  // Removes the oldest timestamp.
  void PopFront() {
    MaybeWriteTimestamp(timestamps_.front());
    timestamps_.pop_front();
  }

  // Writes a timestamp to the file if it is reasonable.
  void MaybeWriteTimestamp(
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
          timestamp);

  std::deque<
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
      timestamps_;

  std::string csv_file_name_;
  FILE *fp_ = nullptr;
  FILE *samples_fp_ = nullptr;

  aos::monotonic_clock::time_point first_time_ = aos::monotonic_clock::min_time;
};

// This class holds 2 NoncausalTimestampFilter's and handles averaging the
// offsets and reporting them.
class NoncausalOffsetEstimator {
 public:
  NoncausalOffsetEstimator(const Node *node_a, const Node *node_b)
      : node_a_(node_a), node_b_(node_b) {}

  // Updates the filter based on a sample from the provided node to the other
  // node.
  void Sample(const Node *node,
              aos::monotonic_clock::time_point node_delivered_time,
              aos::monotonic_clock::time_point other_node_sent_time);

  // Removes old data points from a node before the provided time.
  // Returns true if the line fit changes.
  bool Pop(const Node *node,
           aos::monotonic_clock::time_point node_monotonic_now);

  // Returns a line for the oldest segment.
  Line fit() const { return fit_; }

  // Sets the locations to update when the fit changes.
  void set_offset_pointer(mpq_class *offset_pointer) {
    offset_pointer_ = offset_pointer;
  }
  void set_slope_pointer(mpq_class *slope_pointer) {
    slope_pointer_ = slope_pointer;
  }
  void set_valid_pointer(bool *valid_pointer) {
    valid_pointer_ = valid_pointer;
  }

  // Returns the data points from each filter.
  const std::deque<
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
      &a_timestamps() {
    return a_.timestamps();
  }
  const std::deque<
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
      &b_timestamps() {
    return b_.timestamps();
  }

  void SetFirstFwdTime(monotonic_clock::time_point time) {
    a_.SetFirstTime(time);
  }
  void SetFwdCsvFileName(std::string_view name) { a_.SetCsvFileName(name); }
  void SetFirstRevTime(monotonic_clock::time_point time) {
    b_.SetFirstTime(time);
  }
  void SetRevCsvFileName(std::string_view name) { b_.SetCsvFileName(name); }

  // Logs the fits and timestamps for all the filters.
  void LogFit(std::string_view prefix);

 private:
  void Refit();

  NoncausalTimestampFilter a_;
  NoncausalTimestampFilter b_;

  mpq_class *offset_pointer_ = nullptr;
  mpq_class *slope_pointer_ = nullptr;
  bool *valid_pointer_ = nullptr;

  Line fit_{std::chrono::nanoseconds(0), 0.0};

  const Node *const node_a_;
  const Node *const node_b_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
