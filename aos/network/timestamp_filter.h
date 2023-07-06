#ifndef AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
#define AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>

#include "absl/numeric/int128.h"
#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/time/time.h"

namespace aos {
namespace message_bridge {

// Max velocity to clamp the filter to in seconds/second.
typedef std::ratio<1, 1000> MaxVelocityRatio;
inline constexpr double kMaxVelocity() {
  return static_cast<double>(MaxVelocityRatio::num) /
         static_cast<double>(MaxVelocityRatio::den);
}
inline constexpr std::chrono::nanoseconds kMinNetworkDelay() {
  return std::chrono::nanoseconds(2);
}

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
    VLOG(2) << " " << this << " offset " << offset_;
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
 private:
  struct BootFilter;

 public:
  NoncausalTimestampFilter(const Node *node_a, const Node *node_b)
      : node_a_(node_a), node_b_(node_b) {}

  NoncausalTimestampFilter(NoncausalTimestampFilter &&) noexcept = default;
  NoncausalTimestampFilter &operator=(
      NoncausalTimestampFilter &&other) noexcept {
    // Sigh, std::vector really prefers to copy than move.  We don't want to
    // copy this class or we will end up with double counted samples or put
    // something in the file twice.  The only way it will move instead of copy
    // is if we implement a noexcept move assignment operator.
    node_a_ = other.node_a_;
    other.node_a_ = nullptr;
    node_b_ = other.node_b_;
    other.node_b_ = nullptr;

    filters_ = std::move(other.filters_);
    current_filter_ = other.current_filter_;
    return *this;
  }
  NoncausalTimestampFilter(const NoncausalTimestampFilter &) = delete;
  NoncausalTimestampFilter &operator=(const NoncausalTimestampFilter &) =
      delete;
  ~NoncausalTimestampFilter();

  // A class used to hold all the state information required for identifying a
  // point and caching decisions.
  class Pointer {
   public:
    Pointer() = default;

    Pointer(
        const BootFilter *boot_filter, size_t index,
        std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t0,
        std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t1)
        : boot_filter_(boot_filter), index_(index), t0_(t0), t1_(t1) {}

    // For testing.
    bool operator==(const Pointer &other) const {
      return other.boot_filter_ == boot_filter_ && other.index_ == index_ &&
             other.t0_ == t0_ && other.t1_ == t1_;
    }

   private:
    friend class NoncausalTimestampFilter;

    // The filter which this timestamp came from.
    const BootFilter *boot_filter_ = nullptr;
    // The index for t0 into the timestamps list.
    size_t index_ = 0;
    // The two bounding timestamps.  They will be equal if time last querried
    // time was outside the extents of our timestamps.  These are mostly used to
    // make sure the underlying filter didn't change between when the pointer is
    // created, and re-looked-up.
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t0_ =
        std::make_tuple(monotonic_clock::min_time, std::chrono::nanoseconds(0));
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t1_ =
        std::make_tuple(monotonic_clock::min_time, std::chrono::nanoseconds(0));

    // List of points and their associated times going the other way.
    std::vector<std::pair<size_t, std::tuple<monotonic_clock::time_point,
                                             std::chrono::nanoseconds>>>
        other_points_;
  };

  // Returns the error between the offset in the provided timestamps, the
  // offset at ta, and d error/dta.  Also returns a pointer to the timestamps
  // used for the lookup to be passed back in again for a more efficient second
  // lookup.
  std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
  OffsetError(const NoncausalTimestampFilter *other, Pointer pointer,
              logger::BootTimestamp ta_base, double ta,
              logger::BootTimestamp tb_base, double tb) const {
    const BootFilter *boot_filter = filter(pointer, ta_base.boot, tb_base.boot);
    const SingleFilter *other_filter =
        other == nullptr
            ? nullptr
            : other->maybe_single_filter(tb_base.boot, ta_base.boot);
    std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
        result = boot_filter->filter.OffsetError(
            other_filter, pointer, ta_base.time, ta, tb_base.time, tb);
    result.first.boot_filter_ = boot_filter;
    return result;
  }

  // Returns the error between the offset in the provided timestamps, the
  // bounds offset at ta, and d error/dta.  Also returns a pointer to the
  // timestamps used for the lookup to be passed back in again for a more
  // efficient second lookup.
  std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
  BoundsOffsetError(const NoncausalTimestampFilter *other, Pointer pointer,
                    logger::BootTimestamp ta_base, double ta,
                    logger::BootTimestamp tb_base, double tb) const {
    const BootFilter *boot_filter = filter(pointer, ta_base.boot, tb_base.boot);
    const SingleFilter *other_filter =
        other == nullptr
            ? nullptr
            : other->maybe_single_filter(tb_base.boot, ta_base.boot);
    std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
        result = boot_filter->filter.BoundsOffsetError(
            other_filter, pointer, ta_base.time, ta, tb_base.time, tb);
    result.first.boot_filter_ = boot_filter;
    return result;
  }

  // Returns the string representation of 2 * OffsetError(ta, tb)
  std::string DebugOffsetError(const NoncausalTimestampFilter *other,
                               Pointer pointer, logger::BootTimestamp ta_base,
                               double ta, logger::BootTimestamp tb_base,
                               double tb, size_t node_a, size_t node_b) const;

  // Confirms that the solution meets the constraints.  Returns true on success.
  bool ValidateSolution(const NoncausalTimestampFilter *other, Pointer pointer,
                        logger::BootTimestamp ta, logger::BootTimestamp tb,
                        bool validate_popped, bool quiet) const {
    const SingleFilter *other_filter =
        other == nullptr ? nullptr
                         : other->maybe_single_filter(tb.boot, ta.boot);
    return filter(pointer, ta.boot, tb.boot)
        ->filter.ValidateSolution(other_filter, pointer, ta.time, tb.time,
                                  validate_popped, quiet);
  }
  bool ValidateSolution(const NoncausalTimestampFilter *other, Pointer pointer,
                        logger::BootTimestamp ta_base, double ta,
                        logger::BootTimestamp tb_base, double tb,
                        bool validate_popped, bool quiet) const {
    const SingleFilter *other_filter =
        other == nullptr
            ? nullptr
            : other->maybe_single_filter(tb_base.boot, ta_base.boot);
    return filter(pointer, ta_base.boot, tb_base.boot)
        ->filter.ValidateSolution(other_filter, pointer, ta_base.time, ta,
                                  tb_base.time, tb, validate_popped, quiet);
  }

  // Adds a new sample to our filtered timestamp list.
  void Sample(logger::BootTimestamp monotonic_now,
              logger::BootDuration sample_ns);

  // Removes any old timestamps from our timestamps list.
  // Returns true if any points were popped.
  bool Pop(logger::BootTimestamp time);

  size_t timestamps_size() const {
    size_t result = 0u;
    for (const std::unique_ptr<BootFilter> &filter : filters_) {
      result += filter->filter.timestamps_size();
    }
    return result;
  }

  // Returns the number of timestamps for a specific boot.  This is useful to
  // determine if there are observations in this direction or not.
  size_t timestamps_size(const size_t boota, const size_t bootb) const {
    const BootFilter *f = maybe_filter(boota, bootb);
    if (f == nullptr) {
      return 0u;
    } else {
      return f->filter.timestamps_size();
    }
  }

  // Returns true if there are no timestamps available for the provide boots.
  bool timestamps_empty(const size_t boota, const size_t bootb) const {
    const BootFilter *f = maybe_filter(boota, bootb);
    if (f == nullptr) {
      return true;
    } else {
      return f->filter.timestamps_empty();
    }
  }

  // For testing only:
  void Debug() const {
    for (const std::unique_ptr<BootFilter> &filter : filters_) {
      LOG(INFO) << NodeNames() << " boota: " << filter->boot.first << ", "
                << filter->boot.second;
      filter->filter.Debug();
    }
  }

  // Marks all line segments up until the provided time on the provided node as
  // used.
  void FreezeUntil(logger::BootTimestamp node_monotonic_now,
                   logger::BootTimestamp remote_monotonic_now) {
    // TODO(austin): CHECK that all older boots are fully frozen.
    filter(node_monotonic_now.boot, remote_monotonic_now.boot)
        ->filter.FreezeUntil(node_monotonic_now.time);
    filter(node_monotonic_now.boot, remote_monotonic_now.boot)
        ->filter.FreezeUntilRemote(remote_monotonic_now.time);
  }

  // Returns true if there is a full line which hasn't been observed.
  bool has_unobserved_line() const {
    return filters_.back()->filter.has_unobserved_line();
  }
  // Returns the time of the second point in the unobserved line, or min_time if
  // there is no line.
  logger::BootTimestamp unobserved_line_end() const {
    auto &f = *filters_.back();
    return {static_cast<size_t>(f.boot.first), f.filter.unobserved_line_end()};
  }
  // Returns the time of the second point in the unobserved line on the remote
  // node, or min_time if there is no line.
  logger::BootTimestamp unobserved_line_remote_end() const {
    auto &f = *filters_.back();
    return {static_cast<size_t>(f.boot.second),
            f.filter.unobserved_line_remote_end()};
  }

  // Returns the next timestamp in the queue if available without incrementing
  // the pointer.  This, Consume, and FreezeUntil work together to allow
  // tracking and freezing timestamps which have been combined externally.
  //
  // This doesn't report the virtual points added by the opposite filter
  // because solving for them doesn't add any additional value.  We will already
  // be solving the other direction.
  std::optional<std::tuple<logger::BootTimestamp, logger::BootDuration>>
  Observe() const {
    if (filters_.size() == 0u) {
      return std::nullopt;
    }

    size_t current_filter = std::max(static_cast<ssize_t>(0), current_filter_);
    while (true) {
      const BootFilter &filter = *filters_[current_filter];
      std::optional<
          std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
          result = filter.filter.Observe();
      if (!result) {
        if (current_filter + 1 == filters_.size()) {
          return std::nullopt;
        } else {
          ++current_filter;
          continue;
        }
      }
      return std::make_tuple(
          logger::BootTimestamp{static_cast<size_t>(filter.boot.first),
                                std::get<0>(*result)},
          logger::BootDuration{static_cast<size_t>(filter.boot.second),
                               std::get<1>(*result)});
    }
  }
  // Returns the next timestamp in the queue if available, incrementing the
  // pointer.
  std::optional<std::tuple<logger::BootTimestamp, logger::BootDuration>>
  Consume() {
    if (filters_.size() == 0u) {
      return std::nullopt;
    }
    DCHECK_LT(current_filter_, static_cast<ssize_t>(filters_.size()));

    while (true) {
      std::optional<
          std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
          result =
              current_filter_ < 0 ? std::nullopt
                                  : filters_[current_filter_]->filter.Consume();
      if (!result) {
        if (static_cast<size_t>(current_filter_ + 1) == filters_.size()) {
          return std::nullopt;
        } else {
          ++current_filter_;
          continue;
        }
      }
      BootFilter &filter = *filters_[current_filter_];
      return std::make_tuple(
          logger::BootTimestamp{static_cast<size_t>(filter.boot.first),
                                std::get<0>(*result)},
          logger::BootDuration{static_cast<size_t>(filter.boot.second),
                               std::get<1>(*result)});
    }
  }

  // Public for testing.
  // Assuming that there are at least 2 points in timestamps_, finds the 2
  // matching points.
  std::pair<Pointer,
            std::pair<std::tuple<logger::BootTimestamp, logger::BootDuration>,
                      std::tuple<logger::BootTimestamp, logger::BootDuration>>>
  FindTimestamps(const NoncausalTimestampFilter *other, bool use_other,
                 Pointer pointer, logger::BootTimestamp ta,
                 size_t sample_boot) const {
    const BootFilter *boot_filter = filter(ta.boot, sample_boot);
    std::pair<
        Pointer,
        std::pair<
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>>
        result = boot_filter->filter.FindTimestamps(
            other == nullptr ? nullptr
                             : &other->filter(sample_boot, ta.boot)->filter,
            use_other, pointer, ta.time);
    result.first.boot_filter_ = boot_filter;
    return std::make_pair(
        result.first,
        std::make_pair(
            std::make_tuple(
                logger::BootTimestamp{ta.boot,
                                      std::get<0>(result.second.first)},
                logger::BootDuration{sample_boot,
                                     std::get<1>(result.second.first)}),
            std::make_tuple(
                logger::BootTimestamp{ta.boot,
                                      std::get<0>(result.second.second)},
                logger::BootDuration{sample_boot,
                                     std::get<1>(result.second.second)})));
  }
  std::pair<Pointer,
            std::pair<std::tuple<logger::BootTimestamp, logger::BootDuration>,
                      std::tuple<logger::BootTimestamp, logger::BootDuration>>>
  FindTimestamps(const NoncausalTimestampFilter *other, bool use_other,
                 Pointer pointer, logger::BootTimestamp ta_base, double ta,
                 size_t sample_boot) const;

  static std::chrono::nanoseconds InterpolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p1,
      monotonic_clock::time_point ta);

  static std::chrono::nanoseconds BoundOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p1,
      monotonic_clock::time_point ta);

  static std::chrono::nanoseconds ExtrapolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      monotonic_clock::time_point ta);

  static std::tuple<std::chrono::nanoseconds, double, double> InterpolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p1,
      monotonic_clock::time_point ta_base, double ta);

  static std::tuple<std::chrono::nanoseconds, double, double> BoundOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p1,
      monotonic_clock::time_point ta_base, double ta);

  static std::tuple<std::chrono::nanoseconds, double, double> ExtrapolateOffset(
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
      monotonic_clock::time_point ta_base, double ta);

  const Node *node_a() const { return node_a_; }
  const Node *node_b() const { return node_b_; }

 private:
  // This class holds all the state for the filter for a single pair of boots.
  class SingleFilter {
   public:
    SingleFilter(std::string node_names) : node_names_(std::move(node_names)) {}
    SingleFilter(SingleFilter &&other) noexcept
        : node_names_(std::move(other.node_names_)),
          timestamps_(std::move(other.timestamps_)),
          frozen_time_(other.frozen_time_),
          next_to_consume_(other.next_to_consume_),
          fully_frozen_(other.fully_frozen_),
          has_popped_(other.has_popped_) {}

    SingleFilter &operator=(SingleFilter &&other) noexcept = default;
    SingleFilter(const SingleFilter &) = delete;
    SingleFilter operator=(const SingleFilter &) = delete;
    ~SingleFilter();

    std::pair<
        Pointer,
        std::pair<
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>>
    FindTimestamps(const SingleFilter *other, bool use_other, Pointer pointer,
                   monotonic_clock::time_point ta) const;
    std::pair<
        Pointer,
        std::pair<
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>>
    FindTimestamps(const SingleFilter *other, bool use_other, Pointer pointer,
                   monotonic_clock::time_point ta_base, double ta) const;

    // Check whether the given timestamp falls within our current samples
    bool IsOutsideSamples(monotonic_clock::time_point ta_base, double ta) const;
    // Check whether the given timestamp lies after our current samples
    bool IsAfterSamples(monotonic_clock::time_point ta_base, double ta) const;
    std::pair<Pointer,
              std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
    GetReferenceTimestamp(monotonic_clock::time_point ta_base, double ta) const;

    std::pair<Pointer, std::chrono::nanoseconds> Offset(
        const SingleFilter *other, Pointer pointer,
        monotonic_clock::time_point ta) const;
    std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
    Offset(const SingleFilter *other, Pointer pointer,
           monotonic_clock::time_point ta_base, double ta) const;

    std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
    BoundsOffset(const SingleFilter *other, Pointer pointer,
                 monotonic_clock::time_point ta_base, double ta) const;

    std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
    OffsetError(const SingleFilter *other, Pointer pointer,
                aos::monotonic_clock::time_point ta_base, double ta,
                aos::monotonic_clock::time_point tb_base, double tb) const;

    std::pair<Pointer, std::tuple<std::chrono::nanoseconds, double, double>>
    BoundsOffsetError(const SingleFilter *other, Pointer pointer,
                      aos::monotonic_clock::time_point ta_base, double ta,
                      aos::monotonic_clock::time_point tb_base,
                      double tb) const;

    bool has_unobserved_line() const;
    monotonic_clock::time_point unobserved_line_end() const;
    monotonic_clock::time_point unobserved_line_remote_end() const;
    std::optional<
        std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
    Observe() const;
    std::optional<
        std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
    Consume();
    void FreezeUntil(aos::monotonic_clock::time_point node_monotonic_now);
    void FreezeUntilRemote(
        aos::monotonic_clock::time_point remote_monotonic_now);
    void PopFront();
    void Debug() const;

    // Returns if the timestamp is frozen or not.
    bool frozen(size_t index) const {
      return fully_frozen_ || std::get<0>(timestamps_[index]) <= frozen_time_;
    }

    bool frozen(aos::monotonic_clock::time_point t) const {
      return t <= frozen_time_;
    }

    size_t timestamps_size() const { return timestamps_.size(); }
    bool timestamps_empty() const { return timestamps_.empty(); }

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
        // modify the first sample (rather than remove it) to retain the
        // knowledge of the velocity, but adhere to the constraints.
        //
        // We are doing this here so as points get added in any order, we don't
        // confuse ourselves about what really happened.
        if (absl::int128(doffset.count()) *
                absl::int128(MaxVelocityRatio::den) >
            absl::int128(dt.count()) * absl::int128(MaxVelocityRatio::num)) {
          DCHECK_GE(dt.count(), 0);
          const aos::monotonic_clock::duration adjusted_initial_time =
              std::get<1>(timestamps_[1]) -
              aos::monotonic_clock::duration(
                  static_cast<aos::monotonic_clock::duration::rep>(
                      absl::int128(dt.count()) *
                      absl::int128(MaxVelocityRatio::num) /
                      absl::int128(MaxVelocityRatio::den)));

          return std::make_tuple(std::get<0>(timestamps_[0]),
                                 adjusted_initial_time);
        }
      }
      CHECK_LT(i, timestamps_.size());
      return std::make_tuple(std::get<0>(timestamps_[i]),
                             std::get<1>(timestamps_[i]));
    }
    // Confirms that the solution meets the constraints.  Returns true on
    // success.
    bool ValidateSolution(const SingleFilter *other, Pointer pointer,
                          aos::monotonic_clock::time_point ta,
                          aos::monotonic_clock::time_point tb,
                          bool validate_popped, bool quiet) const;
    bool ValidateSolution(const SingleFilter *other, Pointer pointer,
                          aos::monotonic_clock::time_point ta_base, double ta,
                          aos::monotonic_clock::time_point tb_base, double tb,
                          bool validate_popped, bool quiet) const;

    void Sample(monotonic_clock::time_point monotonic_now,
                std::chrono::nanoseconds sample_ns);

   private:
    std::string node_names_;

    // Timestamp, offest, and then a boolean representing if this sample is
    // frozen and can't be modified or not.
    std::deque<
        std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>>
        timestamps_;

    aos::monotonic_clock::time_point frozen_time_ =
        aos::monotonic_clock::min_time;

    // The index of the next element in timestamps to consume.  0 means none
    // have been consumed, and size() means all have been consumed.
    size_t next_to_consume_ = 0;

    bool fully_frozen_ = false;

    bool has_popped_ = false;
  };

  // Removes the oldest timestamp.
  void PopFront();

  // Writes any saved timestamps to file.
  void FlushSavedSamples();

  const Node *node_a_;
  const Node *node_b_;

  // Returns a debug string with the nodes this filter represents.
  std::string NodeNames() const;

  struct BootFilter {
    BootFilter(std::pair<int, int> new_boot, std::string node_names)
        : boot(new_boot), filter(std::move(node_names)) {}

    BootFilter(BootFilter &&other) noexcept = default;
    BootFilter &operator=(BootFilter &&other) noexcept = default;
    BootFilter(const BootFilter &) = delete;
    void operator=(const BootFilter &) = delete;
    std::pair<int, int> boot;
    SingleFilter filter;
  };

  static bool FilterLessThanUpper(const std::pair<int, int> &l,
                                  const std::unique_ptr<BootFilter> &r) {
    return l < r->boot;
  }
  static bool FilterLessThanLower(const std::unique_ptr<BootFilter> &l,
                                  const std::pair<int, int> &r) {
    return l->boot < r;
  }

 protected:
  BootFilter *filter(int boota, int bootb) {
    auto it =
        std::lower_bound(filters_.begin(), filters_.end(),
                         std::make_pair(boota, bootb), FilterLessThanLower);
    if (it != filters_.end() && (*it)->boot == std::make_pair(boota, bootb)) {
      return it->get();
    }

    if (!filters_.empty() && current_filter_ >= 0) {
      CHECK_LT(static_cast<size_t>(current_filter_), filters_.size());
      CHECK_GE(boota, filters_[current_filter_]->boot.first);
      CHECK_GE(bootb, filters_[current_filter_]->boot.second) << NodeNames();
    }
    BootFilter *result =
        filters_
            .emplace(std::upper_bound(filters_.begin(), filters_.end(),
                                      std::make_pair(boota, bootb),
                                      FilterLessThanUpper),
                     std::make_unique<BootFilter>(std::make_pair(boota, bootb),
                                                  NodeNames()))
            ->get();

    {
      // Confirm we don't have boots go backwards.
      // It is impossible for us to get (0, 0), (0, 1), (1, 0), (1, 1).  That
      // means that both boots on both devices talked to both other boots.
      int last_boota = -1;
      int last_bootb = -1;
      for (const std::unique_ptr<BootFilter> &filter : filters_) {
        CHECK(filter->boot.first != last_boota ||
              filter->boot.second != last_bootb)
            << ": Boots didn't increase.";
        CHECK_GE(filter->boot.first, last_boota);
        CHECK_GE(filter->boot.second, last_bootb);
        last_boota = filter->boot.first;
        last_bootb = filter->boot.second;
      }
    }
    return result;
  }

  const BootFilter *maybe_filter(int boota, int bootb) const {
    auto it =
        std::lower_bound(filters_.begin(), filters_.end(),
                         std::make_pair(boota, bootb), FilterLessThanLower);
    if (it == filters_.end()) {
      return nullptr;
    }
    if (it->get()->boot == std::make_pair(boota, bootb)) {
      return it->get();
    } else {
      return nullptr;
    }
  }

  const BootFilter *maybe_filter(Pointer pointer, int boota, int bootb) const {
    if (pointer.boot_filter_ != nullptr &&
        pointer.boot_filter_->boot.first == static_cast<int>(boota) &&
        pointer.boot_filter_->boot.second == static_cast<int>(bootb)) {
      return pointer.boot_filter_;
    }

    return maybe_filter(boota, bootb);
  }

  const BootFilter *filter(int boota, int bootb) const {
    const BootFilter *result = maybe_filter(boota, bootb);
    CHECK(result != nullptr)
        << NodeNames() << " Failed to find " << boota << ", " << bootb;
    return result;
  }

  const BootFilter *filter(Pointer pointer, int boota, int bootb) const {
    const BootFilter *result = maybe_filter(pointer, boota, bootb);
    CHECK(result != nullptr)
        << NodeNames() << " Failed to find " << boota << ", " << bootb;
    return result;
  }

  static std::pair<
      Pointer,
      std::pair<
          std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>,
          std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>>
  InterpolateWithOtherFilter(
      Pointer pointer, bool use_other, monotonic_clock::time_point ta,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t0,
      std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t1);

  const SingleFilter *maybe_single_filter(int boota, int bootb) const {
    const BootFilter *boot_filter = maybe_filter(boota, bootb);
    return boot_filter == nullptr ? nullptr : &boot_filter->filter;
  }

 private:
  std::vector<std::unique_ptr<BootFilter>> filters_;

  ssize_t current_filter_ = -1;

  // The filter to resume popping from.
  size_t pop_filter_ = 0;
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
  NoncausalOffsetEstimator(NoncausalOffsetEstimator &&) noexcept = default;
  NoncausalOffsetEstimator &operator=(
      NoncausalOffsetEstimator &&other) noexcept = default;
  NoncausalOffsetEstimator(const NoncausalOffsetEstimator &) = delete;
  NoncausalOffsetEstimator &operator=(const NoncausalOffsetEstimator &) =
      delete;

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
  void Sample(const Node *node, logger::BootTimestamp node_delivered_time,
              logger::BootTimestamp other_node_sent_time);
  // Updates the filter for the provided node based on a sample going to the
  // provided node from the other node.
  void ReverseSample(const Node *node, logger::BootTimestamp node_sent_time,
                     logger::BootTimestamp other_node_delivered_time);

 private:
  NoncausalTimestampFilter a_;
  NoncausalTimestampFilter b_;

  const Node *node_a_;
  const Node *node_b_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_TIMESTAMP_FILTER_H_
