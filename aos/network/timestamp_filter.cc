#include "aos/network/timestamp_filter.h"

#include <chrono>
#include <iomanip>
#include <tuple>

#include "absl/numeric/int128.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "aos/configuration.h"
#include "aos/time/time.h"

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;
using logger::BootDuration;
using logger::BootTimestamp;
using Pointer = NoncausalTimestampFilter::Pointer;

std::string TimeString(const aos::monotonic_clock::time_point t,
                       std::chrono::nanoseconds o) {
  std::stringstream ss;
  ss << "O(" << t << ") = " << o.count() << "ns, remote " << t + o;
  return ss.str();
}

std::string TimeString(const aos::monotonic_clock::time_point t_base, double t,
                       std::chrono::nanoseconds o_base, double o) {
  std::stringstream ss;
  ss << "O(" << t_base << ", " << t
     << ") = " << o_base.count() + static_cast<int64_t>(std::round(o)) << "ns, "
     << o - std::round(o) << ", remote "
     << t_base + o_base +
            chrono::nanoseconds(static_cast<int64_t>(std::round(t + o)))
     << "." << t + o - static_cast<int64_t>(std::round(t + o));
  return ss.str();
}

std::string TimeString(
    const std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
        t) {
  return TimeString(std::get<0>(t), std::get<1>(t));
}

void ClippedAverageFilterPrintHeader(FILE *fp) {
  fprintf(fp,
          "# time_since_start, sample_ns, filtered_offset, offset, "
          "velocity, filtered_velocity, velocity_contribution, "
          "sample_contribution, time_contribution\n");
}

void NormalizeTimestamps(monotonic_clock::time_point *ta_base, double *ta) {
  chrono::nanoseconds ta_digits(static_cast<int64_t>(std::floor(*ta)));
  *ta_base += ta_digits;
  *ta -= static_cast<double>(ta_digits.count());

  // Sign, numerical precision wins again.
  //   *ta_base=1000.300249970sec, *ta=-1.35525e-20
  // We then promptly round this to
  //   *ta_base=1000.300249969sec, *ta=1
  // The 1.0 then breaks the LT assumption below, so we kersplat.
  //
  // Detect this case directly and move the 1.0 back into ta_base.
  if (*ta == 1.0) {
    *ta = 0.0;
    *ta_base += chrono::nanoseconds(1);
  }

  CHECK_GE(*ta, 0.0);
  CHECK_LT(*ta, 1.0);
}
void NormalizeTimestamps(BootTimestamp *ta_base, double *ta) {
  NormalizeTimestamps(&ta_base->time, ta);
}

}  // namespace

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
  VLOG(2) << "  " << this << " Sample at " << monotonic_now << " is "
          << sample_ns.count() << "ns, Base is " << base_offset_.count();
  CHECK_GE(monotonic_now, last_time_)
      << ": " << this << " Being asked to filter backwards in time!";
  // Compute the sample offset as a double (seconds), taking into account the
  // base offset.
  const double sample =
      chrono::duration_cast<chrono::duration<double>>(sample_ns - base_offset_)
          .count();

  VelocitySample(monotonic_now, sample_ns);

  // This is our first sample.  Just use it.
  if (last_time_ == aos::monotonic_clock::min_time) {
    VLOG(1) << "  " << this << " First, setting offset to sample.";
    offset_ = sample;
    velocity_contribution_ = 0.0;
    sample_contribution_ = 0.0;
    time_contribution_ = 0.0;
  } else {
    const double dt = chrono::duration_cast<chrono::duration<double>>(
                          monotonic_now - last_time_)
                          .count();
    const double velocity_contribution = dt * filtered_velocity();
    velocity_contribution_ = velocity_contribution;
    offset_ += velocity_contribution;
    // Took less time to transmit, so clamp to it.
    if (sample < offset_) {
      offset_ = sample;
      velocity_contribution_ = 0.0;
      sample_contribution_ = 0.0;
      time_contribution_ = 0.0;
    } else {
      // We split things up into 2 portions.
      //  1) Each sample has information.  Correct some using it.
      //  2) We want to keep a decent time constant if the sample rate slows.
      //     Take time since the last sample into account.
      //
      // There are other challenges.  If every long sample is followed by a
      // short sample, we will over-weight the long samples.
      //
      // In the end, this algorithm does ok for reasonably well conditioned
      // data, but occasionally violates causality when the network delay goes
      // from large to small suddenly.  Good enough for a real time estimate,
      // but not great for where we need 100% accuracy replaying logs.

      // Time constant for the low pass filter in seconds.
      constexpr double kTau = 1.0;

      constexpr double kClampPositive = 0.0005;
      // Scale the slew rate clamp by dt.  This helps when there is an outage.
      const double clamp_positive = kClampPositive * dt;
      clamp_ = clamp_positive;

      {
        // 1)
        constexpr double kAlpha = 0.0005;
        // This formulation is more numerically precise.
        const double sample_contribution =
            std::min(kAlpha * (sample - offset_), 0.000001);
        offset_ = offset_ + sample_contribution;
        sample_contribution_ = sample_contribution;
      }

      {
        // 2)
        //
        // 1-e^(-t/tau) -> alpha
        const double alpha = -std::expm1(-dt / kTau);

        // Clamp to clamp_positive ms to reduce the effect of wildly large
        // samples.
        const double time_contribution =
            std::min(alpha * (sample - offset_), clamp_positive);
        offset_ = offset_ + time_contribution;

        time_contribution_ = time_contribution;
      }

      VLOG(2) << "  " << this << " filter sample is " << offset_;
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
  last_velocity_sample_time_ = aos::monotonic_clock::min_time;
  velocity_ = 0;
  filtered_velocity_ = 0;
}

void TimestampFilter::Reset() {
  offset_ = 0;

  last_time_ = aos::monotonic_clock::min_time;
  base_offset_ = chrono::nanoseconds(0);

  last_velocity_sample_time_ = aos::monotonic_clock::min_time;
}

void TimestampFilter::VelocitySample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  if (last_velocity_sample_time_ == aos::monotonic_clock::min_time) {
    last_velocity_sample_time_ = monotonic_now;
    last_velocity_sample_ns_ = sample_ns;
    velocity_ = 0.0;
    state_velocity_ = 0.0;
    filtered_velocity_ = 0.0;
    last_sample_ns_ = sample_ns;
    filtered_velocity_time_ = 0.5;
  } else {
    chrono::duration<double> elapsed_time =
        chrono::duration_cast<chrono::duration<double>>(
            monotonic_now - last_velocity_sample_time_);

    velocity_ = chrono::duration_cast<chrono::duration<double>>(sample_ns -
                                                                last_sample_ns_)
                    .count() /
                chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                                last_time_)
                    .count();
    if (sample_ns - last_velocity_sample_ns_ <
        chrono::duration_cast<chrono::nanoseconds>(
            chrono::duration<double>(elapsed_time.count() * kMaxVelocity()))) {
      state_velocity_ = chrono::duration_cast<chrono::duration<double>>(
                            sample_ns - last_velocity_sample_ns_)
                            .count() /
                        elapsed_time.count();
      last_velocity_sample_ns_ = sample_ns;
      last_velocity_sample_time_ = monotonic_now;

      constexpr double kSampleTime = 1.0;

      // Limit the weight of historical time.  This makes it so we slow down
      // over time, but don't slow down forever.
      const double clamped_time =
          std::min(kSampleTime, filtered_velocity_time_);

      // Compute a weighted average of the previous velocity and the new
      // velocity.  The filtered velocity is weighted by a time it has been
      // accumulated over, and the sample velocity is purely based on the
      // elapsed time.
      const double unclamped_velocity =
          (filtered_velocity_ * clamped_time +
           std::clamp(state_velocity_, -0.1, kMaxVelocity()) *
               elapsed_time.count()) /
          (clamped_time + elapsed_time.count());

      filtered_velocity_ =
          std::clamp(unclamped_velocity, -kMaxVelocity(), kMaxVelocity());
      filtered_velocity_time_ += elapsed_time.count();
    }
  }
  last_sample_ns_ = sample_ns;
}

void ClippedAverageFilter::SetFwdCsvFileName(std::string_view name) {
  fwd_csv_file_name_ = name;
  fwd_fp_ = fopen(absl::StrCat(fwd_csv_file_name_, ".csv").c_str(), "w");
  ClippedAverageFilterPrintHeader(fwd_fp_);
}

void ClippedAverageFilter::SetRevCsvFileName(std::string_view name) {
  rev_csv_file_name_ = name;
  rev_fp_ = fopen(absl::StrCat(rev_csv_file_name_, ".csv").c_str(), "w");
  ClippedAverageFilterPrintHeader(rev_fp_);
}

void ClippedAverageFilter::set_first_fwd_time(
    aos::monotonic_clock::time_point time) {
  first_fwd_time_ = time;
  if (fwd_fp_) {
    fwd_fp_ = freopen(NULL, "wb", fwd_fp_);
    ClippedAverageFilterPrintHeader(fwd_fp_);
  }
}

void ClippedAverageFilter::set_first_rev_time(
    aos::monotonic_clock::time_point time) {
  first_rev_time_ = time;
  if (rev_fp_) {
    rev_fp_ = freopen(NULL, "wb", rev_fp_);
    ClippedAverageFilterPrintHeader(rev_fp_);
  }
}

void ClippedAverageFilter::FwdSet(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(2) << "Fwd Set";
  fwd_.Set(monotonic_now, sample_ns);
  Update(monotonic_now, &last_fwd_time_);
}

void ClippedAverageFilter::FwdSample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(1) << &fwd_ << " Fwd sample now " << monotonic_now << " sample "
          << sample_ns.count();
  fwd_.Sample(monotonic_now, sample_ns);
  Update(monotonic_now, &last_fwd_time_);

  if (fwd_fp_ != nullptr) {
    if (first_fwd_time_ == aos::monotonic_clock::min_time) {
      first_fwd_time_ = monotonic_now;
    }
    fprintf(
        fwd_fp_,
        "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %9.9f, %9.9f, "
        "%9.9f, %.9f\n",
        chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                        first_fwd_time_)
            .count(),
        chrono::duration_cast<chrono::duration<double>>(sample_ns).count(),
        fwd_.offset() + fwd_.base_offset_double(),
        chrono::duration_cast<chrono::duration<double>>(offset()).count(),
        fwd_.velocity(), fwd_.filtered_velocity(),
        chrono::duration_cast<chrono::duration<double>>(
            monotonic_now.time_since_epoch())
            .count(),
        offset_velocity_, fwd_.last_velocity_sample(),
        fwd_.velocity_contribution(), fwd_.sample_contribution(),
        fwd_.time_contribution(), fwd_.clamp());
    fflush(fwd_fp_);
  }
}

void ClippedAverageFilter::RevSet(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(2) << "Rev set";
  rev_.Set(monotonic_now, sample_ns);
  Update(monotonic_now, &last_rev_time_);
}

void ClippedAverageFilter::RevSample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(1) << "Rev sample";
  rev_.Sample(monotonic_now, sample_ns);
  Update(monotonic_now, &last_rev_time_);

  if (rev_fp_ != nullptr) {
    if (first_rev_time_ == aos::monotonic_clock::min_time) {
      first_rev_time_ = monotonic_now;
    }
    fprintf(
        rev_fp_,
        "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %9.9f, %9.9f, "
        "%9.9f, %.9f\n",
        chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                        first_rev_time_)
            .count(),
        chrono::duration_cast<chrono::duration<double>>(sample_ns).count(),
        rev_.offset() + rev_.base_offset_double(),
        chrono::duration_cast<chrono::duration<double>>(offset()).count(),
        rev_.velocity(), rev_.filtered_velocity(),
        chrono::duration_cast<chrono::duration<double>>(
            monotonic_now.time_since_epoch())
            .count(),
        offset_velocity_, rev_.last_velocity_sample(),
        rev_.velocity_contribution(), rev_.sample_contribution(),
        rev_.time_contribution(), rev_.clamp());
    fflush(rev_fp_);
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

  if (fwd_fp_) {
    fprintf(fwd_fp_, "# Closing and opening\n");
    fclose(fwd_fp_);
    fwd_fp_ = NULL;
    SetFwdCsvFileName(fwd_csv_file_name_);
  }
  if (rev_fp_) {
    fprintf(rev_fp_, "# Closing and opening\n");
    fclose(rev_fp_);
    rev_fp_ = NULL;
    SetRevCsvFileName(rev_csv_file_name_);
  }
}

void ClippedAverageFilter::Reset() {
  base_offset_ = chrono::nanoseconds(0);
  offset_ = 0;
  offset_velocity_ = 0;

  last_fwd_time_ = aos::monotonic_clock::min_time;
  last_rev_time_ = aos::monotonic_clock::min_time;
  first_fwd_time_ = aos::monotonic_clock::min_time;
  first_rev_time_ = aos::monotonic_clock::min_time;

  fwd_.Reset();
  rev_.Reset();

  if (fwd_fp_) {
    fclose(fwd_fp_);
    fwd_fp_ = NULL;
    SetFwdCsvFileName(fwd_csv_file_name_);
  }
  if (rev_fp_) {
    fclose(rev_fp_);
    rev_fp_ = NULL;
    SetRevCsvFileName(rev_csv_file_name_);
  }
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
  VLOG(2) << this << "  Max(fwd) " << hard_max << " min(rev) " << hard_min;
  // We don't want to clip the offset to the hard min/max.  We really want to
  // keep it within a band around the middle.  ratio of 0.3 means stay within
  // +- 0.15 of the middle of the hard min and max.
  constexpr double kBand = 0.3;
  const double max = average + kBand / 2.0 * (hard_max - hard_min);
  const double min = average - kBand / 2.0 * (hard_max - hard_min);

  // Update regardless for the first sample from both the min and max.
  if (*last_time == aos::monotonic_clock::min_time) {
    VLOG(1) << this << "  No last time " << average;
    offset_ = average;
    offset_velocity_ = 0.0;
  } else {
    // Do just a time constant based update.  We can afford to be slow here
    // for smoothness.
    constexpr double kTau = 5.0;
    constexpr double kTauVelocity = 0.75;
    const double dt = chrono::duration_cast<chrono::duration<double>>(
                          monotonic_now - *last_time)
                          .count();
    const double alpha = -std::expm1(-dt / kTau);
    const double velocity_alpha = -std::expm1(-dt / kTauVelocity);

    // Clamp it such that it remains in the min/max bounds.
    offset_ += std::clamp(offset_velocity_, -kMaxVelocity(), kMaxVelocity()) *
               dt / 2.0;
    offset_ = std::clamp(offset_ - alpha * (offset_ - average), min, max);

    offset_velocity_ =
        offset_velocity_ -
        velocity_alpha *
            (offset_velocity_ -
             (fwd_.filtered_velocity() - rev_.filtered_velocity()) / 2.0);

    VLOG(2) << this << "  last time " << offset_;
  }
  *last_time = monotonic_now;

  if (sample_pointer_ != nullptr) {
    // TODO(austin): Probably shouldn't do the update if we don't have fwd and
    // reverse samples.
    if (!MissingSamples()) {
      *sample_pointer_ = offset_;
      VLOG(1) << this << " Updating sample to " << offset_;
    } else {
      VLOG(1) << this << " Don't have both samples.";
      if (last_fwd_time_ == aos::monotonic_clock::min_time) {
        VLOG(1) << this << " Missing forward";
      }
      if (last_rev_time_ == aos::monotonic_clock::min_time) {
        VLOG(1) << this << " Missing reverse";
      }
    }
  }
}

NoncausalTimestampFilter::SingleFilter::~SingleFilter() {}

NoncausalTimestampFilter::~NoncausalTimestampFilter() {}

std::tuple<monotonic_clock::time_point, chrono::nanoseconds> TrimTuple(
    std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds, bool>
        t) {
  return std::make_tuple(std::get<0>(t), std::get<1>(t));
}

std::pair<Pointer, std::pair<std::tuple<BootTimestamp, BootDuration>,
                             std::tuple<BootTimestamp, BootDuration>>>
NoncausalTimestampFilter::FindTimestamps(const NoncausalTimestampFilter *other,
                                         bool use_other, Pointer pointer,
                                         BootTimestamp ta_base, double ta,
                                         size_t sample_boot) const {
  CHECK_GE(ta, 0.0);
  CHECK_LT(ta, 1.0);

  // Since ta is less than an integer, and timestamps should be at least 1 ns
  // apart, we can ignore ta if we make sure that the end of the segment is
  // strictly > than ta_base.
  return FindTimestamps(other, use_other, pointer, ta_base, sample_boot);
}

std::pair<
    Pointer,
    std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
              std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
NoncausalTimestampFilter::SingleFilter::FindTimestamps(
    const SingleFilter *other, bool use_other, Pointer pointer,
    monotonic_clock::time_point ta_base, double ta) const {
  CHECK_GE(ta, 0.0);
  CHECK_LT(ta, 1.0);

  // Since ta is less than an integer, and timestamps should be at least 1 ns
  // apart, we can ignore ta if we make sure that the end of the segment is
  // strictly > than ta_base.
  return FindTimestamps(other, use_other, pointer, ta_base);
}

std::pair<
    Pointer,
    std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
              std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
NoncausalTimestampFilter::InterpolateWithOtherFilter(
    Pointer pointer, bool use_other, monotonic_clock::time_point ta,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> t0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> t1) {
  if (!use_other) {
    return std::make_pair(pointer, std::make_pair(t0, t1));
  }
  // We have 2 timestamps bookending everything, and a list of points in the
  // middle.
  //
  // There are really 3 cases. The time is before the hunk in the middle, after
  // the hunk in the middle, or in the hunk in the middle.
  if (ta <= std::get<0>(pointer.other_points_[0].second)) {
    // We are before the hunk!  Use the start point, and the beginning of the
    // hunk.
    t1 = pointer.other_points_[0].second;
    CHECK_LE(
        absl::int128(std::abs((std::get<1>(t1) - std::get<1>(t0)).count())) *
            absl::int128(MaxVelocityRatio::den),
        absl::int128((std::get<0>(t1) - std::get<0>(t0)).count()) *
            absl::int128(MaxVelocityRatio::num))
        << ": t0 " << TimeString(t0) << ", t1 " << TimeString(t1);
  } else if (ta >
             std::get<0>(pointer.other_points_[pointer.other_points_.size() - 1]
                             .second)) {
    // We are after the hunk!  Use the end point, and the end of the
    // hunk.
    t0 = pointer.other_points_[pointer.other_points_.size() - 1].second;
    CHECK_LE(
        absl::int128(std::abs((std::get<1>(t1) - std::get<1>(t0)).count())) *
            absl::int128(MaxVelocityRatio::den),
        absl::int128((std::get<0>(t1) - std::get<0>(t0)).count()) *
            absl::int128(MaxVelocityRatio::num))
        << ": t0 " << TimeString(t0) << ", t1 " << TimeString(t1);
  } else {
    // We are inside the hunk.  Find the points bounding it.
    CHECK_GT(pointer.other_points_.size(), 1u);

    auto it = std::upper_bound(
        pointer.other_points_.begin() + 1, pointer.other_points_.end() - 1, ta,
        [](monotonic_clock::time_point ta,
           std::pair<size_t, std::tuple<aos::monotonic_clock::time_point,
                                        std::chrono::nanoseconds>>
               t) { return ta < std::get<0>(t.second); });

    t0 = (it - 1)->second;
    t1 = it->second;
  }
  DCHECK_LT(std::get<0>(t0), std::get<0>(t1));
  return std::make_pair(pointer, std::make_pair(t0, t1));
}

std::pair<
    Pointer,
    std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
              std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
NoncausalTimestampFilter::SingleFilter::FindTimestamps(
    const SingleFilter *other, bool use_other, Pointer pointer,
    monotonic_clock::time_point ta) const {
  CHECK_GT(timestamps_size(), 1u);

  std::tuple<monotonic_clock::time_point, chrono::nanoseconds> t0;
  std::tuple<monotonic_clock::time_point, chrono::nanoseconds> t1;

  // boot_filter_ is non-null when the rest of the contents are valid.  Make
  // sure it's pointing to this filter, and the pointer is in bounds.
  if (pointer.boot_filter_ != nullptr &&
      &pointer.boot_filter_->filter == this &&
      pointer.index_ + 1 != timestamps_size()) {
    CHECK_LT(pointer.index_ + 1, timestamps_size()) << " " << this;
    // Confirm that the cached timestamps haven't changed so we can trust the
    // results.
    //
    // TODO(austin): Should this be a DCHECK when we are happier?  This is a
    // constraint on the user's behavior we are enforcing.
    CHECK(timestamp(pointer.index_) == pointer.t0_)
        << ": " << this << " boot_filter " << pointer.boot_filter_ << " got "
        << std::get<0>(timestamp(pointer.index_)) << ", "
        << std::get<1>(timestamp(pointer.index_)).count()
        << "ns, expected index " << pointer.index_ << ", "
        << std::get<0>(pointer.t0_) << ", " << std::get<1>(pointer.t0_).count()
        << "ns";
    // Last point has been filtered out.
    if (pointer.t0_ != pointer.t1_) {
      // If t0 and t1 match, this was a "before the start" point.  We can still
      // check it against the first segment, but we know it won't match so don't
      // enforce the CHECK that the cache matches.
      CHECK(timestamp(pointer.index_ + 1) == pointer.t1_)
          << ": " << this << " boot_filter " << pointer.boot_filter_
          << " index " << pointer.index_ << ", size " << timestamps_size()
          << ", got " << std::get<0>(timestamp(pointer.index_ + 1)) << ", "
          << std::get<1>(timestamp(pointer.index_ + 1)).count()
          << "ns, expected index " << pointer.index_ << ", "
          << std::get<0>(pointer.t1_) << ", "
          << std::get<1>(pointer.t1_).count() << "ns";
    }

    t0 = timestamp(pointer.index_);
    if (ta >= std::get<0>(t0)) {
      t1 = timestamp(pointer.index_ + 1);
      if (ta < std::get<0>(t1)) {
        if (pointer.other_points_.empty()) {
          return std::make_pair(pointer, std::make_pair(t0, t1));
        }

        // Er, we shouldn't be able to have a non-empty other_points_ without
        // having other and points...
        CHECK(other != nullptr);
        CHECK(!other->timestamps_empty());

        // TODO(austin): Is there a cheaper way to verify nothing has changed?
        // Should we add a generation counter of some sort?
        for (const auto &point : pointer.other_points_) {
          const auto other_point = other->timestamps_[point.first];
          CHECK(std::get<0>(other_point) + std::get<1>(other_point) ==
                std::get<0>(point.second))
              << ": Cache changed";
        }

        return InterpolateWithOtherFilter(pointer, use_other, ta, t0, t1);
      }
    }
  }

  // Otherwise, do a log(n) search from the starting point.  We should be close.
  // TODO(austin): We should be able to do a better search here if we've got a
  // previous pointer.  Searches tend to be close to the previous search.

  auto it = std::upper_bound(
      timestamps_.begin() + 1, timestamps_.end() - 1, ta,
      [](monotonic_clock::time_point ta,
         std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
             t) { return ta < std::get<0>(t); });

  const size_t index = std::distance(timestamps_.begin(), it);

  pointer.index_ = index - 1;
  t0 = timestamp(index - 1);
  pointer.t0_ = t0;
  t1 = timestamp(index);
  pointer.t1_ = t1;

  if (other != nullptr && !other->timestamps_empty()) {
    // Ok, we now need to find all points within our range in the matched
    // filter.
    auto other_t0_it =
        std::lower_bound(other->timestamps_.begin(), other->timestamps_.end(),
                         std::get<0>(pointer.t0_),
                         [](std::tuple<aos::monotonic_clock::time_point,
                                       std::chrono::nanoseconds>
                                t,
                            monotonic_clock::time_point ta) {
                           return ta > std::get<0>(t) + std::get<1>(t);
                         });
    if (other_t0_it != other->timestamps_.end()) {
      auto other_t1_it = std::upper_bound(
          other_t0_it, other->timestamps_.end(), std::get<0>(pointer.t1_),
          [](monotonic_clock::time_point ta,
             std::tuple<aos::monotonic_clock::time_point,
                        std::chrono::nanoseconds>
                 t) { return ta < std::get<0>(t) + std::get<1>(t); });

      if (std::get<0>(*other_t0_it) + std::get<1>(*other_t0_it) <
          std::get<0>(pointer.t1_)) {
        pointer.other_points_.clear();

        // Now, we've got a range.  [other_t0_it, other_t1_it).
        for (auto other_it = other_t0_it; other_it != other_t1_it; ++other_it) {
          const std::tuple<monotonic_clock::time_point,
                           std::chrono::nanoseconds>
              flipped_point = std::make_tuple(
                  std::get<0>(*other_it) + std::get<1>(*other_it),
                  -std::get<1>(*other_it) - kMinNetworkDelay());

          // If the new point from the opposite direction filter is below the
          // interpolated value at that point, then the opposite direction point
          // defines a new min and we should take it.
          if (NoncausalTimestampFilter::InterpolateOffset(
                  pointer.t0_, pointer.t1_, std::get<0>(flipped_point)) >
              std::get<1>(flipped_point)) {
            // Add it to the list of points to consider.
            pointer.other_points_.emplace_back(std::make_pair(
                std::distance(other->timestamps_.begin(), other_it),
                flipped_point));
          }
        }

        if (pointer.other_points_.size() > 0) {
          return InterpolateWithOtherFilter(pointer, use_other, ta, t0, t1);
        }
      }
    }

    // other_t0_it will always be > t0, even if it is at the end.
    //   1) other_t0_it < t0
    //   2) other_t0_it < t1
    //
    // t0_it will always be > x.
  }
  return std::make_pair(pointer, std::make_pair(t0, t1));
}

std::pair<Pointer, std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
NoncausalTimestampFilter::SingleFilter::GetReferenceTimestamp(
    monotonic_clock::time_point ta_base, double ta) const {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  std::tuple<monotonic_clock::time_point, chrono::nanoseconds>
      reference_timestamp = timestamp(0);

  // TODO(austin): Return more here.  Whether we are the first or last, and the
  // corresponding index.
  if (ta_base >= std::get<0>(timestamp(timestamps_size() - 1u))) {
    reference_timestamp = timestamp(timestamps_size() - 1u);
    return std::make_pair(Pointer(nullptr, timestamps_size() - 1u,
                                  reference_timestamp, reference_timestamp),
                          reference_timestamp);
  } else {
    return std::make_pair(
        Pointer(nullptr, 0, reference_timestamp, reference_timestamp),
        reference_timestamp);
  }
}

bool NoncausalTimestampFilter::SingleFilter::IsOutsideSamples(
    monotonic_clock::time_point ta_base, double ta) const {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  if (timestamps_size() == 1u || ta_base < std::get<0>(timestamp(0)) ||
      ta_base >= std::get<0>(timestamp(timestamps_size() - 1u))) {
    return true;
  }

  return false;
}

bool NoncausalTimestampFilter::SingleFilter::IsAfterSamples(
    monotonic_clock::time_point ta_base, double ta) const {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  if (ta_base >= std::get<0>(timestamp(timestamps_size() - 1u))) {
    return true;
  }

  return false;
}

chrono::nanoseconds NoncausalTimestampFilter::ExtrapolateOffset(
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
    monotonic_clock::time_point ta) {
  return ExtrapolateOffset(p0, ta, 0.0).first;
}

chrono::nanoseconds NoncausalTimestampFilter::InterpolateOffset(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p1,
    monotonic_clock::time_point ta) {
  return InterpolateOffset(p0, p1, ta, 0.0).first;
}

std::pair<chrono::nanoseconds, double>
NoncausalTimestampFilter::InterpolateOffset(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p1,
    monotonic_clock::time_point ta_base, double ta) {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);

  // Given 2 points defining a line and the time along that line, interpolate.
  //
  // ta may be massive, but the points will be close, so compute everything
  // relative to the points.  2 64 bit numbers multiplied together could result
  // in a 128 bit number, so let's use one to be safe.  Multiply before divide
  // and use 128 bit arithmetic to make this perfectly precise in integer math.
  //
  //  oa = p0.o + (ta - p0.t) * (p1.o - p0.o) / (p1.t - p0.t)
  //
  // Add (or subtract, integer division rounds towards 0...) 0.5 ((dt / 2) / dt)
  // to the numerator to round to the nearest number rather than round down.
  const chrono::nanoseconds time_in = ta_base - std::get<0>(p0);
  const chrono::nanoseconds dt = std::get<0>(p1) - std::get<0>(p0);
  const chrono::nanoseconds doffset = std::get<1>(p1) - std::get<1>(p0);

  absl::int128 numerator =
      absl::int128(time_in.count()) * absl::int128(doffset.count());
  numerator += numerator > 0 ? absl::int128(dt.count() / 2)
                             : -absl::int128(dt.count() / 2);

  const chrono::nanoseconds integer =
      std::get<1>(p0) + chrono::nanoseconds(static_cast<int64_t>(
                            numerator / absl::int128(dt.count())));
  // Compute the remainder of the division in InterpolateOffset above, and
  // then use double math to compute it accurately.  Since integer math rounds
  // down, we need to undo the rounding to get the double remainder.  Add or
  // subtract dt/2/dt (0.5) to undo the addition.
  //
  // We have good tests which confirm for small offsets this matches nicely. For
  // large offsets, the 128 bit math will take care of us.
  const double remainder =
      static_cast<double>(numerator % absl::int128(dt.count())) / dt.count() +
      (numerator > 0 ? -0.5 : 0.5) +
      ta * static_cast<double>(doffset.count()) /
          static_cast<double>(dt.count());
  return std::make_pair(integer, remainder);
}

chrono::nanoseconds NoncausalTimestampFilter::BoundOffset(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p1,
    monotonic_clock::time_point ta) {
  // We are trying to solve for worst case offset given the two known points.
  // This is on the two worst case lines from the two points, and we switch
  // lines at the interstection.  This is equivilent to the lowest of the two
  // lines.
  return std::max(NoncausalTimestampFilter::ExtrapolateOffset(p0, ta),
                  NoncausalTimestampFilter::ExtrapolateOffset(p1, ta));
}

std::pair<chrono::nanoseconds, double> NoncausalTimestampFilter::BoundOffset(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p1,
    monotonic_clock::time_point ta_base, double ta) {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);

  const std::pair<chrono::nanoseconds, double> o0 =
      NoncausalTimestampFilter::ExtrapolateOffset(p0, ta_base, ta);
  const std::pair<chrono::nanoseconds, double> o1 =
      NoncausalTimestampFilter::ExtrapolateOffset(p1, ta_base, ta);

  // Want to calculate max(o0 + o0r, o1 + o1r) without precision problems.
  if (static_cast<double>((o0.first - o1.first).count()) >
      o1.second - o0.second) {
    // Ok, o0 is now > o1.  We want the max, so return o0.
    return o0;
  } else {
    return o1;
  }
}

std::pair<chrono::nanoseconds, double>
NoncausalTimestampFilter::ExtrapolateOffset(
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
    monotonic_clock::time_point ta_base, double ta) {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  // Since the point (p0) is an integer, we now can guarantee that ta won't put
  // us on a different side of p0.  This is because ta is between 0 and 1, and
  // always positive.  Compute the integer and double portions and return them.
  const chrono::nanoseconds dt = ta_base - std::get<0>(p0);

  if (dt < std::chrono::nanoseconds(0)) {
    // Extrapolate backwards, using the (positive) MaxVelocity slope
    // We've been asked to extrapolate the offset to a time before our first
    // sample point.  To be conservative, we'll return an extrapolated
    // offset that is less than (less tight an estimate of the network delay)
    // than our sample offset, bound by the max slew velocity we allow
    //       p0
    //      /
    //     /
    //   ta
    // Since dt < 0, we shift by dt * slope to get that value
    //
    // Take the remainder of the math in ExtrapolateOffset above and compute it
    // with floating point math.  Our tests are good enough to confirm that this
    // works as designed.
    const absl::int128 numerator =
        (absl::int128(dt.count() - MaxVelocityRatio::den / 2) *
         absl::int128(MaxVelocityRatio::num));
    return std::make_pair(
        std::get<1>(p0) + chrono::nanoseconds(static_cast<int64_t>(
                              numerator / absl::int128(MaxVelocityRatio::den))),
        static_cast<double>(numerator % absl::int128(MaxVelocityRatio::den)) /
                static_cast<double>(MaxVelocityRatio::den) +
            0.5 + ta * kMaxVelocity());
  } else {
    // Extrapolate forwards, using the (negative) MaxVelocity slope
    // Same concept, except going foward past our last (most recent) sample:
    //       pN
    //         |
    //          |
    //           ta
    // Since dt > 0, we shift by - dt * slope to get that value
    //
    // Take the remainder of the math in ExtrapolateOffset above and compute it
    // with floating point math.  Our tests are good enough to confirm that this
    // works as designed.
    const absl::int128 numerator =
        absl::int128(dt.count() + MaxVelocityRatio::den / 2) *
        absl::int128(MaxVelocityRatio::num);
    return std::make_pair(
        std::get<1>(p0) - chrono::nanoseconds(static_cast<int64_t>(
                              numerator / absl::int128(MaxVelocityRatio::den))),
        -static_cast<double>(numerator % absl::int128(MaxVelocityRatio::den)) /
                static_cast<double>(MaxVelocityRatio::den) +
            0.5 - ta * kMaxVelocity());
  }
}

std::pair<Pointer, chrono::nanoseconds>
NoncausalTimestampFilter::SingleFilter::Offset(
    const SingleFilter *other, Pointer pointer,
    monotonic_clock::time_point ta) const {
  CHECK_GT(timestamps_size(), 0u);
  if (IsOutsideSamples(ta, 0.)) {
    // Special case when size = 1 or if we're asked to extrapolate to
    // times before or after we have data.
    auto reference_timestamp = GetReferenceTimestamp(ta, 0.);
    return std::make_pair(reference_timestamp.first,
                          NoncausalTimestampFilter::ExtrapolateOffset(
                              reference_timestamp.second, ta));
  }

  std::pair<
      Pointer,
      std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
                std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
      points = FindTimestamps(other, true, pointer, ta);
  return std::make_pair(points.first,
                        NoncausalTimestampFilter::InterpolateOffset(
                            points.second.first, points.second.second, ta));
}

std::pair<Pointer, std::pair<chrono::nanoseconds, double>>
NoncausalTimestampFilter::SingleFilter::Offset(
    const SingleFilter *other, Pointer pointer,
    monotonic_clock::time_point ta_base, double ta) const {
  CHECK_GT(timestamps_size(), 0u) << node_names_;
  if (IsOutsideSamples(ta_base, ta)) {
    // Special case size = 1 or ta_base before first timestamp or
    // after last timesteamp, so we need to extrapolate out
    std::pair<Pointer,
              std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
        reference_timestamp = GetReferenceTimestamp(ta_base, ta);
    return std::make_pair(reference_timestamp.first,
                          NoncausalTimestampFilter::ExtrapolateOffset(
                              reference_timestamp.second, ta_base, ta));
  }

  std::pair<
      Pointer,
      std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
                std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
      points = FindTimestamps(other, true, pointer, ta_base, ta);
  CHECK_LT(std::get<0>(points.second.first), std::get<0>(points.second.second));
  // Return both the integer and double portion together to save a timestamp
  // lookup.
  return std::make_pair(
      points.first,
      NoncausalTimestampFilter::InterpolateOffset(
          points.second.first, points.second.second, ta_base, ta));
}

std::pair<Pointer, std::pair<chrono::nanoseconds, double>>
NoncausalTimestampFilter::SingleFilter::BoundsOffset(
    const SingleFilter *other, Pointer pointer,
    monotonic_clock::time_point ta_base, double ta) const {
  CHECK_GT(timestamps_size(), 0u) << node_names_;
  if (IsOutsideSamples(ta_base, ta)) {
    // Special case size = 1 or ta_base before first timestamp or
    // after last timestamp, so we need to extrapolate out
    std::pair<Pointer,
              std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
        reference_timestamp = GetReferenceTimestamp(ta_base, ta);
    return std::make_pair(reference_timestamp.first,
                          NoncausalTimestampFilter::ExtrapolateOffset(
                              reference_timestamp.second, ta_base, ta));
  }

  std::pair<
      Pointer,
      std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
                std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
      points = FindTimestamps(other, false, pointer, ta_base, ta);
  CHECK_LT(std::get<0>(points.second.first), std::get<0>(points.second.second));
  // Return both the integer and double portion together to save a timestamp
  // lookup.
  return std::make_pair(points.first, NoncausalTimestampFilter::BoundOffset(
                                          points.second.first,
                                          points.second.second, ta_base, ta));
}

std::pair<Pointer, std::pair<chrono::nanoseconds, double>>
NoncausalTimestampFilter::SingleFilter::OffsetError(
    const SingleFilter *other, Pointer pointer,
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  const std::pair<Pointer, std::pair<std::chrono::nanoseconds, double>> offset =
      Offset(other, pointer, ta_base, ta);

  // Compute the integer portion first, and the double portion second.  Subtract
  // the results of each.  This handles large offsets without losing precision.
  return std::make_pair(
      offset.first, std::make_pair(((tb_base - ta_base) - offset.second.first),
                                   (tb - ta) - offset.second.second));
}

std::pair<Pointer, std::pair<chrono::nanoseconds, double>>
NoncausalTimestampFilter::SingleFilter::BoundsOffsetError(
    const SingleFilter *other, Pointer pointer,
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  const std::pair<Pointer, std::pair<std::chrono::nanoseconds, double>> offset =
      BoundsOffset(other, pointer, ta_base, ta);

  // Compute the integer portion first, and the double portion second.  Subtract
  // the results of each.  This handles large offsets without losing precision.
  return std::make_pair(
      offset.first, std::make_pair((tb_base - ta_base) - offset.second.first,
                                   (tb - ta) - offset.second.second));
}

std::string NoncausalTimestampFilter::DebugOffsetError(
    const NoncausalTimestampFilter *other, Pointer pointer,
    BootTimestamp ta_base, double ta, BootTimestamp tb_base, double tb,
    size_t node_a, size_t node_b) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  const BootFilter *f = maybe_filter(pointer, ta_base.boot, tb_base.boot);
  if (f == nullptr || f->filter.timestamps_size() == 0u) {
    return "0";
  }

  if (f->filter.IsOutsideSamples(ta_base.time, ta)) {
    auto reference_timestamp =
        f->filter.GetReferenceTimestamp(ta_base.time, ta);
    double slope = kMaxVelocity();
    std::string note = "_";
    if (f->filter.IsAfterSamples(ta_base.time, ta)) {
      slope = -kMaxVelocity();
      note = "^";
    }
    // d cost / dtb ==> 2 * OffsetError(ta, tb) ==>
    // 2 * (tb - ta - (ta - ref) * ma - ref_offset)
    return absl::StrFormat(
        "2. * (t%d - t%d - ((t%d - %d) * %f + %d)%s", node_b, node_a, node_a,
        std::get<0>(reference_timestamp.second).time_since_epoch().count(),
        slope, std::get<1>(reference_timestamp.second).count(), note);
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = f->filter
                   .FindTimestamps(
                       other == nullptr
                           ? nullptr
                           : &other->filter(tb_base.boot, ta_base.boot)->filter,
                       true, pointer, ta_base.time, ta)
                   .second;

  // As a reminder, our cost function is essentially:
  //   ((tb - ta - (ma ta + ba))^2
  // ie
  //   ((tb - (1 + ma) ta - ba)^2
  //
  // d cost/dtb => 2 * OffsetError(ta, tb) ==>
  //   2 * ((tb - (1 + ma) ta - ba)

  const int64_t rise =
      (std::get<1>(points.second) - std::get<1>(points.first)).count();
  const int64_t run =
      (std::get<0>(points.second) - std::get<0>(points.first)).count();

  if (rise == 0) {
    return absl::StrFormat("2. * (t%d - t%d %c %d.)", node_b, node_a,
                           std::get<1>(points.first).count() < 0 ? '+' : '-',
                           std::abs(std::get<1>(points.first).count()));
  }

  return absl::StrFormat("2. * (t%d - t%d - (t%d - %d.) * %d. / %d. - %d.)",
                         node_b, node_a, node_a,
                         std::get<0>(points.first).time_since_epoch().count(),
                         rise, run, std::get<1>(points.first).count());
}

std::string NoncausalTimestampFilter::NodeNames() const {
  return absl::StrCat(node_a_->name()->string_view(), " -> ",
                      node_b_->name()->string_view());
}

bool NoncausalTimestampFilter::SingleFilter::ValidateSolution(
    const SingleFilter *other, Pointer pointer,
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);
  CHECK_GT(timestamps_size(), 0u);
  if (ta_base < std::get<0>(timestamp(0)) && has_popped_) {
    LOG(ERROR) << node_names_ << " O(" << ta_base << ", " << ta
               << ") is before the start and we have forgotten the answer.";
    return false;
  }
  if (IsOutsideSamples(ta_base, ta)) {
    // Special case size = 1 or ta_base before first timestamp or
    // after last timestamp, so we need to extrapolate out
    auto reference_timestamp = GetReferenceTimestamp(ta_base, ta);

    // Special case size = 1 or ta before first timestamp, so we extrapolate
    const std::pair<chrono::nanoseconds, double> offset =
        NoncausalTimestampFilter::ExtrapolateOffset(reference_timestamp.second,
                                                    ta_base, ta);

    // We want to do offset + ta > tb, but we need to do it with minimal
    // numerical precision problems.
    // See below for why this is a >=
    if (static_cast<double>((offset.first + ta_base - tb_base).count()) >=
        tb - ta - offset.second) {
      LOG(ERROR) << node_names_ << " "
                 << TimeString(ta_base, ta, offset.first, offset.second)
                 << " > solution time "
                 << tb_base + chrono::nanoseconds(
                                  static_cast<int64_t>(std::round(tb)))
                 << ", " << tb - std::round(tb) << " foo";
      LOG(INFO) << "Remainder " << offset.second;
      return false;
    }
    return true;
  }

  // Honestly, here, we care about confirming that the worst case holds.  This
  // means that each solution is plausible based on the points that we have. The
  // only thing we actually know is that time will slew by at most the max slew
  // rate, so the candidate solution must be within the max slew rate from the
  // samples.
  std::pair<
      Pointer,
      std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
                std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
      points = FindTimestamps(other, false, pointer, ta_base, ta);
  const std::pair<chrono::nanoseconds, double> offset =
      NoncausalTimestampFilter::BoundOffset(points.second.first,
                                            points.second.second, ta_base, ta);
  // See below for why this is a >=
  if (static_cast<double>((offset.first + ta_base - tb_base).count()) >=
      tb - offset.second - ta) {
    LOG(ERROR) << node_names_ << " "
               << TimeString(ta_base, ta, offset.first, offset.second)
               << " > solution time " << tb_base << ", " << tb;
    LOG(ERROR) << "Bracketing times are " << TimeString(points.second.first)
               << " and " << TimeString(points.second.second);
    return false;
  }
  return true;
}

bool NoncausalTimestampFilter::SingleFilter::ValidateSolution(
    const SingleFilter *other, Pointer pointer,
    aos::monotonic_clock::time_point ta,
    aos::monotonic_clock::time_point tb) const {
  CHECK_GT(timestamps_size(), 0u);
  if (ta < std::get<0>(timestamp(0)) && has_popped_) {
    LOG(ERROR) << node_names_ << " O(" << ta
               << ") is before the start and we have forgotten the answer.";
    return false;
  }

  // The logic here mirrors the double variant above almost perfectly.  See
  // above for the comments.

  if (IsOutsideSamples(ta, 0.)) {
    auto reference_timestamp = GetReferenceTimestamp(ta, 0.);

    const chrono::nanoseconds offset =
        NoncausalTimestampFilter::ExtrapolateOffset(reference_timestamp.second,
                                                    ta);
    // Note: this needs to be >=.  The simulation code doesn't give us a good
    // way to preserve order well enough to have causality preserved when things
    // happen at the same point in time.
    if (offset + ta >= tb) {
      LOG(ERROR) << node_names_ << " " << TimeString(ta, offset)
                 << " > solution time " << tb;
      return false;
    }
    return true;
  }

  std::pair<
      Pointer,
      std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
                std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>>
      points = FindTimestamps(other, false, pointer, ta);
  const chrono::nanoseconds offset = NoncausalTimestampFilter::BoundOffset(
      points.second.first, points.second.second, ta);

  // Note: this needs to be >=.  The simulation code doesn't give us a good
  // way to preserve order well enough to have causality preserved when things
  // happen at the same point in time.
  if (offset + ta >= tb) {
    LOG(ERROR) << node_names_ << " " << TimeString(ta, offset)
               << " > solution time " << tb;
    LOG(ERROR) << "Bracketing times are " << TimeString(points.second.first)
               << " and " << TimeString(points.second.second);
    return false;
  }
  return true;
}

void NoncausalTimestampFilter::Sample(BootTimestamp monotonic_now_all,
                                      BootDuration sample_ns) {
  filter(monotonic_now_all.boot, sample_ns.boot)
      ->filter.Sample(monotonic_now_all.time, sample_ns.duration);
}

void NoncausalTimestampFilter::SingleFilter::Sample(
    monotonic_clock::time_point monotonic_now, chrono::nanoseconds sample_ns) {
  // The first sample is easy.  Just do it!
  if (timestamps_.size() == 0) {
    VLOG(1) << node_names_ << " Initial sample of "
            << TimeString(monotonic_now, sample_ns);
    timestamps_.emplace_back(std::make_tuple(monotonic_now, sample_ns));
    CHECK(!fully_frozen_)
        << ": " << node_names_
        << " Returned a horizontal line previously and then "
           "got a new sample at "
        << monotonic_now << ", "
        << chrono::duration<double>(monotonic_now - std::get<0>(timestamps_[0]))
               .count()
        << " seconds after the last sample at " << std::get<0>(timestamps_[0])
        << ".  Increase --time_estimation_buffer_seconds to greater than "
        << chrono::duration<double>(monotonic_now - std::get<0>(timestamps_[0]))
               .count();
    return;
  }
  CHECK_GT(monotonic_now, frozen_time_)
      << ": " << node_names_ << " Tried to insert " << monotonic_now
      << " before the frozen time of " << frozen_time_
      << ".  Increase "
         "--time_estimation_buffer_seconds to greater than "
      << chrono::duration<double>(frozen_time_ - monotonic_now).count();

  // Future samples get quite a bit harder.  We want the line to track the
  // highest point without volating the slope constraint.
  std::tuple<aos::monotonic_clock::time_point, chrono::nanoseconds> back =
      timestamps_.back();

  aos::monotonic_clock::duration dt = monotonic_now - std::get<0>(back);
  aos::monotonic_clock::duration doffset = sample_ns - std::get<1>(back);

  if (dt == chrono::nanoseconds(0) && doffset == chrono::nanoseconds(0)) {
    VLOG(1) << node_names_ << " Duplicate sample of O(" << monotonic_now
            << ") = " << sample_ns.count() << ", remote time "
            << monotonic_now + sample_ns;

    return;
  }

  // Lets handle the easy case first.  We are just appending.
  if (dt > chrono::nanoseconds(0)) {
    // We are trying to draw a line through the most positive points which
    // adheres to our +- velocity constraint. If the point is less than the max
    // negative slope, the point violates our constraint and will never be worth
    // considering.  Ignore it.
    if (absl::int128(doffset.count()) * absl::int128(MaxVelocityRatio::den) <
        -absl::int128(dt.count()) * absl::int128(MaxVelocityRatio::num)) {
      VLOG(1) << std::setprecision(1) << std::fixed << node_names_
              << " Rejected sample of " << TimeString(monotonic_now, sample_ns)
              << " because " << doffset.count() << " < "
              << (-dt * kMaxVelocity()).count() << " len "
              << timestamps_.size();
      return;
    }

    // Be overly conservative here.  It either won't make a difference, or
    // will give us an error with an actual useful time difference.
    CHECK(!fully_frozen_)
        << ": " << node_names_
        << " Returned a horizontal line previously and then got a new "
           "sample at "
        << monotonic_now << ", "
        << chrono::duration<double>(monotonic_now - std::get<0>(timestamps_[0]))
               .count()
        << " seconds after the last sample at " << std::get<0>(timestamps_[0])
        << ".  Increase --time_estimation_buffer_seconds to greater than "
        << chrono::duration<double>(monotonic_now - std::get<0>(timestamps_[0]))
               .count();

    // Back propagate the max velocity and remove any elements violating the
    // velocity constraint.  This is to handle the case where the offsets were
    // becoming gradually more negative, and then there's a sudden positive
    // jump.
    //
    // 1      4
    //    2
    //       3
    //
    // In this case, point 3 is now violating our constraint and we need to
    // remove it.  This is the non-causal part of the filter.
    while (absl::int128(dt.count()) * absl::int128(MaxVelocityRatio::num) <
               absl::int128(doffset.count()) *
                   absl::int128(MaxVelocityRatio::den) &&
           timestamps_.size() > 1u) {
      CHECK(!frozen(std::get<0>(back)))
          << ": " << node_names_ << " Can't pop an already frozen sample "
          << TimeString(back) << " while inserting "
          << TimeString(monotonic_now, sample_ns) << ", "
          << chrono::duration<double>(monotonic_now - std::get<0>(back)).count()
          << " seconds in the past.  Increase --time_estimation_buffer_seconds "
             "to greater than "
          << chrono::duration<double>(monotonic_now - std::get<0>(back))
                 .count();
      VLOG(1) << node_names_
              << " Removing now invalid sample during back propegation of "
              << TimeString(back);
      timestamps_.pop_back();

      back = timestamps_.back();
      dt = monotonic_now - std::get<0>(back);
      doffset = sample_ns - std::get<1>(back);
    }

    VLOG(1) << node_names_ << " Added sample of "
            << TimeString(monotonic_now, sample_ns);
    timestamps_.emplace_back(std::make_tuple(monotonic_now, sample_ns));
    return;
  }

  // Since it didn't fit at the end, now figure out where to insert our new
  // point.  lower_bound returns the element which we are supposed to insert
  // "before".
  auto it = std::lower_bound(
      timestamps_.begin(), timestamps_.end(), monotonic_now,
      [](const std::tuple<aos::monotonic_clock::time_point,
                          std::chrono::nanoseconds>
             x,
         monotonic_clock::time_point t) { return std::get<0>(x) < t; });

  CHECK(it != timestamps_.end());

  // We shouldn't hit this one, but I really want to be sure...
  CHECK(!frozen(std::get<0>(*(it))));

  if (it == timestamps_.begin()) {
    // We are being asked to add at the beginning.
    {
      const chrono::nanoseconds dt = std::get<0>(*it) - monotonic_now;
      const chrono::nanoseconds original_offset = std::get<1>(*it);
      const chrono::nanoseconds doffset = original_offset - sample_ns;

      if (dt == chrono::nanoseconds(0) && doffset >= chrono::nanoseconds(0)) {
        VLOG(1) << node_names_ << " Redundant timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(timestamps_.front())
                << " is at the same time and a better solution.";
        return;
      }
    }

    VLOG(1) << node_names_ << " Added sample at beginning "
            << TimeString(monotonic_now, sample_ns);
    timestamps_.insert(it, std::make_tuple(monotonic_now, sample_ns));

    while (true) {
      // First point was too positive, so we need to remove points after it
      // until we are valid.
      auto second = timestamps_.begin() + 1;
      if (second != timestamps_.end()) {
        const chrono::nanoseconds dt = std::get<0>(*second) - monotonic_now;
        const chrono::nanoseconds doffset = std::get<1>(*second) - sample_ns;

        if (absl::int128(doffset.count()) *
                absl::int128(MaxVelocityRatio::den) <
            -absl::int128(dt.count()) * absl::int128(MaxVelocityRatio::num)) {
          VLOG(1) << node_names_ << " Removing redundant sample of "
                  << TimeString(*second) << " because "
                  << TimeString(timestamps_.front())
                  << " would make the slope too negative.";
          timestamps_.erase(second);
          continue;
        }

        auto third = second + 1;
        if (third != timestamps_.end()) {
          // The second point might need to be popped.  This shows up when
          // the first point violated the constraints, but in timestamp(), we
          // were clipping it to be valid.  When a point is added before it, the
          // prior first point might now be invalid and need to be cleaned up.
          //
          //    3
          //
          // 1 2
          //
          // Point 2 was invalid before, but was clipped in timestamp(), but can
          // now be removed.
          const chrono::nanoseconds dt =
              std::get<0>(*third) - std::get<0>(*second);
          const chrono::nanoseconds doffset =
              std::get<1>(*third) - std::get<1>(*second);

          if (absl::int128(doffset.count()) *
                  absl::int128(MaxVelocityRatio::den) >
              absl::int128(dt.count()) * absl::int128(MaxVelocityRatio::num)) {
            VLOG(1) << node_names_ << " Removing invalid sample of "
                    << TimeString(*second) << " because " << TimeString(*third)
                    << " would make the slope too positive.";
            timestamps_.erase(second);
            continue;
          }
        }
      }

      break;
    }
    return;
  } else {
    VLOG(1) << node_names_ << " Found the next time " << std::get<0>(*(it - 1))
            << " < " << monotonic_now << " < " << std::get<0>(*it);

    {
      chrono::nanoseconds prior_dt = monotonic_now - std::get<0>(*(it - 1));
      chrono::nanoseconds prior_doffset = sample_ns - std::get<1>(*(it - 1));
      chrono::nanoseconds next_dt = std::get<0>(*it) - monotonic_now;
      chrono::nanoseconds next_doffset = std::get<1>(*it) - sample_ns;

      // If we are worse than either the previous or next point, discard.
      if (absl::int128(prior_doffset.count()) *
              absl::int128(MaxVelocityRatio::den) <
          absl::int128(-prior_dt.count()) *
              absl::int128(MaxVelocityRatio::num)) {
        VLOG(1) << node_names_ << " Ignoring timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(*(it - 1))
                << " is before and the slope would be too negative.";
        return;
      }
      if (absl::int128(next_doffset.count()) *
              absl::int128(MaxVelocityRatio::den) >
          absl::int128(next_dt.count()) * absl::int128(MaxVelocityRatio::num)) {
        VLOG(1) << node_names_ << " Ignoring timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(*it)
                << " is following and the slope would be too positive.";
        return;
      }

      if ((prior_dt == chrono::nanoseconds(0) &&
           prior_doffset == chrono::nanoseconds(0)) ||
          (next_dt == chrono::nanoseconds(0) &&
           next_doffset == chrono::nanoseconds(0))) {
        VLOG(1) << node_names_ << " Ignoring timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(*it) << " matches one of the points.";
        return;
      }
    }

    // Now, insert and start propagating forwards and backwards anything we've
    // made invalid.  Do this simultaneously so we keep discovering anything
    // new.
    auto middle_it =
        timestamps_.insert(it, std::make_tuple(monotonic_now, sample_ns));
    VLOG(1) << node_names_ << " Inserted " << TimeString(*middle_it);

    while (middle_it != timestamps_.end() && middle_it != timestamps_.begin()) {
      auto next_it =
          (middle_it == timestamps_.end()) ? timestamps_.end() : middle_it + 1;
      auto prior_it = (middle_it == timestamps_.begin()) ? timestamps_.begin()
                                                         : middle_it - 1;

      // See if the next can be popped.  If so, pop it.
      if (next_it != timestamps_.end()) {
        const chrono::nanoseconds next_dt =
            std::get<0>(*next_it) - std::get<0>(*middle_it);
        const chrono::nanoseconds next_doffset =
            std::get<1>(*next_it) - std::get<1>(*middle_it);

        if (absl::int128(next_doffset.count()) *
                absl::int128(MaxVelocityRatio::den) <
            absl::int128(-next_dt.count()) *
                absl::int128(MaxVelocityRatio::num)) {
          VLOG(1) << node_names_
                  << " Next slope is too negative, removing next point "
                  << TimeString(*next_it);
          next_it = timestamps_.erase(next_it);
          // erase invalidates all iterators, and this code uses middle as the
          // state.  Update middle.
          middle_it = next_it - 1;
          continue;
        }
      }

      // See if the previous point can be popped.
      if (prior_it != timestamps_.begin()) {
        const chrono::nanoseconds prior_dt =
            std::get<0>(*middle_it) - std::get<0>(*prior_it);
        const chrono::nanoseconds prior_doffset =
            std::get<1>(*middle_it) - std::get<1>(*prior_it);

        if (absl::int128(prior_doffset.count()) *
                absl::int128(MaxVelocityRatio::den) >
            absl::int128(prior_dt.count()) *
                absl::int128(MaxVelocityRatio::num)) {
          CHECK(!frozen(std::get<0>(*prior_it)))
              << ": " << node_names_
              << " Can't pop an already frozen sample.  Increase "
                 "--time_estimation_buffer_seconds to greater than "
              << chrono::duration<double>(prior_dt).count();

          VLOG(1) << "Prior slope is too positive, removing prior point "
                  << TimeString(*prior_it);
          prior_it = timestamps_.erase(prior_it);
          middle_it = prior_it;
          continue;
        }
      }
      // Made no modifications, bail.
      break;
    }
  }
}

bool NoncausalTimestampFilter::Pop(BootTimestamp time) {
  CHECK_GE(filters_.size(), 1u);

  VLOG(1) << NodeNames() << " Pop(" << time << ")";
  bool removed = false;
  while (true) {
    CHECK_LT(pop_filter_, filters_.size());
    BootFilter *boot_filter = filters_[pop_filter_].get();
    CHECK(boot_filter != nullptr);
    size_t timestamps_size = 0;
    while ((timestamps_size = boot_filter->filter.timestamps_size()) > 2) {
      // When the timestamp which is the end of the line is popped, we want to
      // drop it off the list.  Hence the <
      if (time < BootTimestamp{
                     .boot = static_cast<size_t>(boot_filter->boot.first),
                     .time = std::get<0>(boot_filter->filter.timestamp(1))}) {
        return removed;
      }
      boot_filter->filter.PopFront();
      removed = true;
    }

    if (timestamps_size == 2) {
      if (pop_filter_ + 1u >= filters_.size()) {
        return removed;
      }

      // There is 1 more filter, see if there is enough data in it to switch
      // over to it.
      if (filters_[pop_filter_ + 1]->filter.timestamps_size() < 2u) {
        return removed;
      }
      if (time <
          BootTimestamp{.boot = static_cast<size_t>(boot_filter->boot.first),
                        .time = std::get<0>(
                            filters_[pop_filter_ + 1]->filter.timestamp(1))}) {
        return removed;
      }
    }
    VLOG(1) << NodeNames() << " Incrementing pop filter";
    ++pop_filter_;
    if (pop_filter_ == filters_.size()) {
      return removed;
    }
  }
}

void NoncausalTimestampFilter::SingleFilter::Debug() const {
  size_t count = 0;
  for (std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
           timestamp : timestamps_) {
    LOG(INFO) << node_names_ << " "
              << TimeString(std::get<0>(timestamp), std::get<1>(timestamp))
              << " frozen? " << frozen(std::get<0>(timestamp)) << " consumed? "
              << (count < next_to_consume_);
    ++count;
  }
}

monotonic_clock::time_point
NoncausalTimestampFilter::SingleFilter::unobserved_line_end() const {
  if (has_unobserved_line()) {
    return std::get<0>(timestamp(next_to_consume_ + 1));
  }
  return monotonic_clock::min_time;
}

monotonic_clock::time_point
NoncausalTimestampFilter::SingleFilter::unobserved_line_remote_end() const {
  if (has_unobserved_line()) {
    const std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t =
        timestamp(next_to_consume_ + 1);
    return std::get<0>(t) + std::get<1>(t);
  }
  return monotonic_clock::min_time;
}

bool NoncausalTimestampFilter::SingleFilter::has_unobserved_line() const {
  return next_to_consume_ + 1 < timestamps_.size();
}

std::optional<std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
NoncausalTimestampFilter::SingleFilter::Observe() const {
  if (timestamps_.empty() || next_to_consume_ >= timestamps_.size()) {
    return std::nullopt;
  }
  VLOG(1) << node_names_ << " Observed sample of "
          << TimeString(timestamp(next_to_consume_));
  return timestamp(next_to_consume_);
}

std::optional<std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
NoncausalTimestampFilter::SingleFilter::Consume() {
  if (timestamps_.empty() || next_to_consume_ >= timestamps_.size()) {
    return std::nullopt;
  }

  auto result = timestamp(next_to_consume_);
  VLOG(1) << node_names_ << " Consumed sample of " << TimeString(result);
  ++next_to_consume_;
  return result;
}

void NoncausalTimestampFilter::SingleFilter::FreezeUntil(
    aos::monotonic_clock::time_point node_monotonic_now) {
  if (node_monotonic_now < frozen_time_) {
    return;
  }
  for (size_t i = 0; i < timestamps_.size(); ++i) {
    // Freeze 1 point past the match.
    if (std::get<0>(timestamp(i)) >= node_monotonic_now) {
      frozen_time_ = std::get<0>(timestamp(i));
      return;
    }
  }

  if (timestamps_.empty()) {
    VLOG(1) << node_names_ << " fully_frozen_, no timestamps.";
    fully_frozen_ = true;
  } else if (node_monotonic_now > std::get<0>(timestamps_.back())) {
    // We've been asked to freeze past the last point.  It isn't safe to add any
    // more points or we will change this region.
    VLOG(1) << node_names_ << " fully_frozen_, after the end.";
    fully_frozen_ = true;
  } else {
    LOG(FATAL) << "How did we get here?";
  }
}

void NoncausalTimestampFilter::SingleFilter::FreezeUntilRemote(
    aos::monotonic_clock::time_point remote_monotonic_now) {
  for (size_t i = 0; i < timestamps_.size(); ++i) {
    // Freeze 1 point past the match.
    if (std::get<0>(timestamp(i)) + std::get<1>(timestamp(i)) >=
        remote_monotonic_now) {
      frozen_time_ = std::max(std::get<0>(timestamp(i)), frozen_time_);
      return;
    }
  }

  if (timestamps_.empty()) {
    VLOG(1) << node_names_ << " fully_frozen_, no timestamps.";
    fully_frozen_ = true;
  } else if (remote_monotonic_now > std::get<0>(timestamps_.back()) +
                                        std::get<1>(timestamps_.back())) {
    // We've been asked to freeze past the last point.  It isn't safe to add any
    // more points or we will change this region.
    VLOG(1) << node_names_ << " fully_frozen_, after the end.";
    fully_frozen_ = true;
  } else {
    LOG(FATAL) << "How did we get here?";
  }
}

void NoncausalTimestampFilter::SingleFilter::PopFront() {
  // If we drop data, we shouldn't add anything before that point.
  frozen_time_ = std::max(frozen_time_, std::get<0>(timestamp(0)));
  timestamps_.pop_front();
  has_popped_ = true;
  if (next_to_consume_ > 0u) {
    next_to_consume_--;
  }
}

void NoncausalOffsetEstimator::Sample(const Node *node,
                                      BootTimestamp node_delivered_time,
                                      BootTimestamp other_node_sent_time) {
  VLOG(1) << "Sample delivered         " << node_delivered_time << " sent "
          << other_node_sent_time << " " << node->name()->string_view()
          << " -> "
          << ((node == node_a_) ? node_b_ : node_a_)->name()->string_view();
  if (node == node_a_) {
    a_.Sample(node_delivered_time,
              {other_node_sent_time.boot,
               other_node_sent_time.time - node_delivered_time.time});
  } else if (node == node_b_) {
    b_.Sample(node_delivered_time,
              {other_node_sent_time.boot,
               other_node_sent_time.time - node_delivered_time.time});
  } else {
    LOG(FATAL) << "Unknown node " << node->name()->string_view();
  }
}

void NoncausalOffsetEstimator::ReverseSample(
    const Node *node, BootTimestamp node_sent_time,
    BootTimestamp other_node_delivered_time) {
  VLOG(1) << "Reverse sample delivered " << other_node_delivered_time
          << " sent " << node_sent_time << " "
          << ((node == node_a_) ? node_b_ : node_a_)->name()->string_view()
          << " -> " << node->name()->string_view();
  if (node == node_a_) {
    b_.Sample(other_node_delivered_time,
              {node_sent_time.boot,
               node_sent_time.time - other_node_delivered_time.time});
  } else if (node == node_b_) {
    a_.Sample(other_node_delivered_time,
              {node_sent_time.boot,
               node_sent_time.time - other_node_delivered_time.time});
  } else {
    LOG(FATAL) << "Unknown node " << node->name()->string_view();
  }
}

}  // namespace message_bridge
}  // namespace aos
