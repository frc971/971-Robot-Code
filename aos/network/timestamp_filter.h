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

namespace aos {
namespace message_bridge {

// TODO<jim>: Should do something to help with precision, like make it an
// integer and divide by the value (e.g., / 1000)

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
// All the offset and error calculation functions are designed to be used in an
// optimization problem with large times but to retain sub-nanosecond precision.
// To do this, time is treated as both a large integer portion, and a small
// double portion.  All the functions are subtracting the large times in integer
// precision and handling the remainder with double precision.
class NoncausalTimestampFilter {
 public:
  NoncausalTimestampFilter(const Node *node_a, const Node *node_b)
      : node_a_(node_a), node_b_(node_b) {}
  ~NoncausalTimestampFilter();

  // Check whether the given timestamp falls within our current samples
  bool IsOutsideSamples(monotonic_clock::time_point ta_base,
                        double ta) const;

  // Check whether the given timestamp lies after our current samples
  bool IsAfterSamples(monotonic_clock::time_point ta_base, double ta) const;

  std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>
  GetReferenceTimestamp(monotonic_clock::time_point ta_base, double ta) const;

  // Returns the offset for the point in time, using the timestamps in the deque
  // to form a polyline used to interpolate.
  std::chrono::nanoseconds Offset(monotonic_clock::time_point ta) const;
  std::pair<std::chrono::nanoseconds, double> Offset(
      monotonic_clock::time_point ta_base, double ta) const;

  // Returns the error between the offset in the provided timestamps, and the
  // offset at ta.
  double OffsetError(aos::monotonic_clock::time_point ta_base, double ta,
                     aos::monotonic_clock::time_point tb_base, double tb) const;

  // Returns the cost (OffsetError^2), ie (ob - oa - offset(oa, ob))^2,
  // calculated accurately.
  // Since this is designed to be used with a gradient based solver, it isn't
  // super important if Cost is precise.
  double Cost(aos::monotonic_clock::time_point ta_base, double ta,
              aos::monotonic_clock::time_point tb_base, double tb) const;
  std::string DebugCost(aos::monotonic_clock::time_point ta_base, double ta,
                        aos::monotonic_clock::time_point tb_base, double tb,
                        size_t node_a, size_t node_b) const;

  // Returns the partial derivitive dcost/dta
  double DCostDta(aos::monotonic_clock::time_point ta_base, double ta,
                  aos::monotonic_clock::time_point tb_base, double tb) const;
  std::string DebugDCostDta(aos::monotonic_clock::time_point ta_base, double ta,
                            aos::monotonic_clock::time_point tb_base, double tb,
                            size_t node_a, size_t node_b) const;
  // Returns the partial derivitive dcost/dtb
  double DCostDtb(aos::monotonic_clock::time_point ta_base, double ta,
                  aos::monotonic_clock::time_point tb_base, double tb) const;
  std::string DebugDCostDtb(aos::monotonic_clock::time_point ta_base, double ta,
                            aos::monotonic_clock::time_point tb_base, double tb,
                            size_t node_a, size_t node_b) const;

  // Confirms that the solution meets the constraints.  Returns true on success.
  bool ValidateSolution(aos::monotonic_clock::time_point ta,
                        aos::monotonic_clock::time_point tb) const;

  double Convert(double ta) const {
    return ta +
           static_cast<double>(
               Offset(monotonic_clock::epoch(), ta).first.count()) +
           Offset(monotonic_clock::epoch(), ta).second;
  }

  // Adds a new sample to our filtered timestamp list.
  void Sample(aos::monotonic_clock::time_point monotonic_now,
              std::chrono::nanoseconds sample_ns);

  // Removes any old timestamps from our timestamps list.
  // Returns true if any points were popped.
  bool Pop(aos::monotonic_clock::time_point time);

  std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
  timestamp(size_t i) const {
    if (i == 0u && timestamps_.size() >= 2u && !has_popped_) {
      std::chrono::nanoseconds dt =
          std::get<0>(timestamps_[1]) - std::get<0>(timestamps_[0]);
      std::chrono::nanoseconds doffset =
          std::get<1>(timestamps_[1]) - std::get<1>(timestamps_[0]);

      // If we are early in the log file, the filter hasn't had time to get
      // started.  We might only have 2 samples, and the first sample was
      // incredibly delayed, violating our velocity constraint.  In that case,
      // modify the first sample (rather than remove it) to retain the knowledge
      // of the velocity, but adhere to the constraints.
      //
      // We are doing this here so as points get added in any order, we don't
      // confuse ourselves about what really happened.
      if (doffset > dt * kMaxVelocity()) {
        const aos::monotonic_clock::duration adjusted_initial_time =
            std::get<1>(timestamps_[1]) -
            aos::monotonic_clock::duration(
                static_cast<aos::monotonic_clock::duration::rep>(
                    dt.count() * kMaxVelocity()));

        return std::make_tuple(std::get<0>(timestamps_[0]),
                               adjusted_initial_time);
      }
    }
    return std::make_tuple(std::get<0>(timestamps_[i]),
                           std::get<1>(timestamps_[i]));
  }

  // Returns if the timestamp is frozen or not.
  bool frozen(size_t index) const {
    return fully_frozen_ || std::get<2>(timestamps_[index]);
  }

  size_t timestamps_size() const { return timestamps_.size(); }

  void Debug() {
    size_t count = 0;
    for (std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds,
                    bool>
             timestamp : timestamps_) {
      LOG(INFO) << std::get<0>(timestamp) << " offset "
                << std::get<1>(timestamp).count() << " frozen? "
                << std::get<2>(timestamp) << " consumed? "
                << (count < next_to_consume_);
      ++count;
    }
  }

  // Sets the starting point and filename to log samples to.  These functions
  // are only used when doing CSV file logging to debug the filter.
  void SetFirstTime(aos::monotonic_clock::time_point time);
  void SetCsvFileName(std::string_view name);

  // Marks all line segments up until the provided time on the provided node as
  // used.
  void FreezeUntil(aos::monotonic_clock::time_point node_monotonic_now);
  void FreezeUntilRemote(aos::monotonic_clock::time_point remote_monotonic_now);

  // Returns the next timestamp in the queue if available without incrementing
  // the pointer.  This, Consume, and FreezeUntil work together to allow
  // tracking and freezing timestamps which have been combined externally.
  std::optional<
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
  Observe() const;
  // Returns the next timestamp in the queue if available, incrementing the
  // pointer.
  std::optional<
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
  Consume();

  // Public for testing.
  // Assuming that there are at least 2 points in timestamps_, finds the 2
  // matching points.
  std::pair<std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
  FindTimestamps(monotonic_clock::time_point ta) const;
  std::pair<std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
  FindTimestamps(monotonic_clock::time_point ta_base, double ta) const;

  static std::chrono::nanoseconds InterpolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p1,
      monotonic_clock::time_point ta);

  static std::chrono::nanoseconds ExtrapolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      monotonic_clock::time_point ta);

  static std::chrono::nanoseconds InterpolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p1,
      monotonic_clock::time_point ta_base, double ta);

  static double InterpolateOffsetRemainder(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> /*p0*/,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> /*p1*/,
      monotonic_clock::time_point ta_base, double ta);

  static std::chrono::nanoseconds ExtrapolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      monotonic_clock::time_point /*ta_base*/, double /*ta*/);

  static double ExtrapolateOffsetRemainder(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      monotonic_clock::time_point ta_base, double ta);

 private:
  // Removes the oldest timestamp.
  void PopFront();

  // Writes a timestamp to the file if it is reasonable.
  void MaybeWriteTimestamp(
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
          timestamp);

  // Writes any saved timestamps to file.
  void FlushSavedSamples();

  const Node *const node_a_;
  const Node *const node_b_;

  // Timestamp, offest, and then a boolean representing if this sample is frozen
  // and can't be modified or not.
  // TODO(austin): Actually use and update the bool.
  std::deque<std::tuple<aos::monotonic_clock::time_point,
                        std::chrono::nanoseconds, bool>>
      timestamps_;

  // The index of the next element in timestamps to consume.  0 means none have
  // been consumed, and size() means all have been consumed.
  size_t next_to_consume_ = 0;

  // Holds any timestamps from before the start of the log to be flushed when we
  // know when the log starts.
  std::vector<
      std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
      saved_samples_;

  std::string csv_file_name_;
  FILE *fp_ = nullptr;
  FILE *samples_fp_ = nullptr;

  bool fully_frozen_ = false;

  bool has_popped_ = false;

  aos::monotonic_clock::time_point first_time_ = aos::monotonic_clock::min_time;
};

// This class holds 2 NoncausalTimestampFilter's and handles averaging the
// offsets and reporting them.
class NoncausalOffsetEstimator {
 public:
  NoncausalOffsetEstimator(const Node *node_a, const Node *node_b)
      : a_(node_a, node_b),
        b_(node_b, node_a),
        node_a_(node_a),
        node_b_(node_b) {}

  NoncausalTimestampFilter *GetFilter(const Node *n) {
    if (n == node_a_) {
      return &a_;
    }
    if (n == node_b_) {
      return &b_;
    }

    LOG(FATAL) << "Unknown node";
    return nullptr;
  }

  // Updates the filter for the provided node based on a sample from the
  // provided node to the other node.
  void Sample(const Node *node,
              aos::monotonic_clock::time_point node_delivered_time,
              aos::monotonic_clock::time_point other_node_sent_time);
  // Updates the filter for the provided node based on a sample going to the
  // provided node from the other node.
  void ReverseSample(
      const Node *node, aos::monotonic_clock::time_point node_sent_time,
      aos::monotonic_clock::time_point other_node_delivered_time);

  // Removes old data points from a node before the provided time.
  // Returns true if any points were popped.
  bool Pop(const Node *node,
           aos::monotonic_clock::time_point node_monotonic_now);

  // Returns the data points from each filter.
  size_t a_timestamps_size() const { return a_.timestamps_size(); }
  size_t b_timestamps_size() const { return b_.timestamps_size(); }

  void SetFirstFwdTime(monotonic_clock::time_point time) {
    a_.SetFirstTime(time);
  }
  void SetFwdCsvFileName(std::string_view name) { a_.SetCsvFileName(name); }
  void SetFirstRevTime(monotonic_clock::time_point time) {
    b_.SetFirstTime(time);
  }
  void SetRevCsvFileName(std::string_view name) { b_.SetCsvFileName(name); }

 private:
  NoncausalTimestampFilter a_;
  NoncausalTimestampFilter b_;

  const Node *const node_a_;
  const Node *const node_b_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
