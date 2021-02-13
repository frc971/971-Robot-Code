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

std::string TimeString(const aos::monotonic_clock::time_point t,
                       std::chrono::nanoseconds o) {
  std::stringstream ss;
  ss << "O(" << t << ") = " << o.count() << "ns, remote " << t + o;
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

void PrintNoncausalTimestampFilterHeader(FILE *fp) {
  fprintf(fp, "time_since_start,sample_ns,filtered_offset\n");
}

void PrintNoncausalTimestampFilterSamplesHeader(FILE *fp) {
  fprintf(fp,
          "time_since_start,sample_ns,monotonic,monotonic+offset(remote)\n");
}

void NormalizeTimestamps(monotonic_clock::time_point *ta_base, double *ta) {
  chrono::nanoseconds ta_digits(static_cast<int64_t>(std::floor(*ta)));
  *ta_base += ta_digits;
  *ta -= static_cast<double>(ta_digits.count());

  CHECK_GE(*ta, 0.0);
  CHECK_LT(*ta, 1.0);
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
  VLOG(1) << "  " << this << " Sample at " << monotonic_now << " is "
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

      VLOG(1) << "  " << this << " filter sample is " << offset_;
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
  VLOG(1) << "Fwd Set";
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
  VLOG(1) << "Rev set";
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
  VLOG(1) << this << "  Max(fwd) " << hard_max << " min(rev) " << hard_min;
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

    VLOG(1) << this << "  last time " << offset_;
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

NoncausalTimestampFilter::~NoncausalTimestampFilter() {
  // Destroy the filter by popping until empty.  This will trigger any
  // timestamps to be written to the files.
  while (timestamps_.size() != 0u) {
    PopFront();
  }
  if (fp_) {
    fclose(fp_);
  }

  if (samples_fp_) {
    fclose(samples_fp_);
  }
}

std::tuple<monotonic_clock::time_point, chrono::nanoseconds> TrimTuple(
    std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds, bool>
        t) {
  return std::make_tuple(std::get<0>(t), std::get<1>(t));
}

void NoncausalTimestampFilter::FlushSavedSamples() {
  for (const std::tuple<aos::monotonic_clock::time_point,
                        std::chrono::nanoseconds> &sample : saved_samples_) {
    fprintf(samples_fp_, "%.9f, %.9f, %.9f, %.9f\n",
            chrono::duration_cast<chrono::duration<double>>(
                std::get<0>(sample) - first_time_)
                .count(),
            chrono::duration_cast<chrono::duration<double>>(std::get<1>(sample))
                .count(),
            chrono::duration_cast<chrono::duration<double>>(
                std::get<0>(sample).time_since_epoch())
                .count(),
            chrono::duration_cast<chrono::duration<double>>(
                (std::get<0>(sample) + std::get<1>(sample)).time_since_epoch())
                .count());
  }
  fflush(samples_fp_);
  saved_samples_.clear();
}

std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
          std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
NoncausalTimestampFilter::FindTimestamps(monotonic_clock::time_point ta_base,
                                         double ta) const {
  CHECK_GE(ta, 0.0);
  CHECK_LT(ta, 1.0);

  // Since ta is less than an integer, and timestamps should be at least 1 ns
  // apart, we can ignore ta if we make sure that the end of the segment is
  // strictly > than ta_base.
  return FindTimestamps(ta_base);
}

std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
          std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
NoncausalTimestampFilter::FindTimestamps(monotonic_clock::time_point ta) const {
  CHECK_GT(timestamps_size(), 1u);
  auto it = std::upper_bound(
      timestamps_.begin() + 1, timestamps_.end() - 1, ta,
      [](monotonic_clock::time_point ta,
         std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
             t) { return ta < std::get<0>(t); });

  const size_t index = std::distance(timestamps_.begin(), it);

  return std::make_pair(timestamp(index - 1), timestamp(index));
}

chrono::nanoseconds NoncausalTimestampFilter::ExtrapolateOffset(
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
    monotonic_clock::time_point ta) {
  const chrono::nanoseconds dt = ta - std::get<0>(p0);
  if (dt <= std::chrono::nanoseconds(0)) {
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
    return std::get<1>(p0) +
           chrono::duration_cast<chrono::nanoseconds>(dt * kMaxVelocity());
  } else {
    // Extrapolate forwards, using the (negative) MaxVelocity slope
    // Same concept, except going foward past our last (most recent) sample:
    //       pN
    //         |
    //          |
    //           ta
    // Since dt > 0, we shift by - dt * slope to get that value
    return std::get<1>(p0) -
           chrono::duration_cast<chrono::nanoseconds>(dt * kMaxVelocity());
  }
}

chrono::nanoseconds NoncausalTimestampFilter::InterpolateOffset(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p1,
    monotonic_clock::time_point ta) {
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
  const chrono::nanoseconds time_in = ta - std::get<0>(p0);
  const chrono::nanoseconds dt = std::get<0>(p1) - std::get<0>(p0);

  absl::int128 numerator =
      absl::int128(time_in.count()) *
      absl::int128((std::get<1>(p1) - std::get<1>(p0)).count());
  numerator += numerator > 0 ? absl::int128(dt.count() / 2)
                             : -absl::int128(dt.count() / 2);
  return std::get<1>(p0) + chrono::nanoseconds(static_cast<int64_t>(
                               numerator / absl::int128(dt.count())));
}

chrono::nanoseconds NoncausalTimestampFilter::InterpolateOffset(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> /*p1*/,
    monotonic_clock::time_point /*ta_base*/, double /*ta*/) {
  // For the double variant, we want to split the result up into a large integer
  // portion, and the rest.  We want to do this without introducing numerical
  // precision problems.
  //
  // One way would be to carefully compute the integer portion, and then compute
  // the double portion in such a way that the two are guaranteed to add up
  // correctly.
  //
  // The simpler way is to simply just use the offset from p0 as the integer
  // portion, and make the rest be the double portion.  It will get us most of
  // the way there for a lot less work, and we can revisit if this breaks down.
  //
  // oa = p0.o + (ta - p0.t) * (p1.o - p0.o) / (p1.t - p0.t)
  //      ^^^^
  // TODO(austin): Use 128 bit math and the remainder to be more accurate here.
  return std::get<1>(p0);
}

double NoncausalTimestampFilter::InterpolateOffsetRemainder(
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p0,
    std::tuple<monotonic_clock::time_point, chrono::nanoseconds> p1,
    monotonic_clock::time_point ta_base, double ta) {
  const chrono::nanoseconds time_in = ta_base - std::get<0>(p0);
  const chrono::nanoseconds dt = std::get<0>(p1) - std::get<0>(p0);

  // The remainder then is the rest of the equation.
  //
  // oa = p0.o + (ta - p0.t) * (p1.o - p0.o) / (p1.t - p0.t)
  //             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // TODO(austin): Use 128 bit math and the remainder to be more accurate here.
  return static_cast<double>(ta + time_in.count()) /
         static_cast<double>(dt.count()) *
         (std::get<1>(p1) - std::get<1>(p0)).count();
}

chrono::nanoseconds NoncausalTimestampFilter::ExtrapolateOffset(
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
    monotonic_clock::time_point /*ta_base*/, double /*ta*/) {
  // For this version, use the base offset from p0 as the base for the offset
  return std::get<1>(p0);
}

double NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
    std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> p0,
    monotonic_clock::time_point ta_base, double ta) {
  // Compute the remainder portion of this offset
  // oa = p0.o +/- ((ta + ta_base) - p0.t)) * kMaxVelocity()
  //               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // But compute (ta + ta_base - p0.t) as (ta + (ta_base - p0.t))
  // to handle numerical precision
  const chrono::nanoseconds time_in = ta_base - std::get<0>(p0);
  const double dt = static_cast<double>(ta + time_in.count());
  if (dt < 0.0) {
    // Extrapolate backwards with max (positive) slope (which means
    // the returned offset should be negative)
    return dt * kMaxVelocity();
  } else {
    // Extrapolate forwards with max (negative) slope
    return -dt * kMaxVelocity();
  }
}

bool NoncausalTimestampFilter::IsOutsideSamples(
    monotonic_clock::time_point ta_base, double ta) const {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  if (timestamps_size() == 1u || ta_base < std::get<0>(timestamp(0)) ||
      ta_base >= std::get<0>(timestamp(timestamps_size() - 1u))) {
    return true;
  }

  return false;
}

bool NoncausalTimestampFilter::IsAfterSamples(
    monotonic_clock::time_point ta_base, double ta) const {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  if (ta_base >= std::get<0>(timestamp(timestamps_size() - 1u))) {
    return true;
  }

  return false;
}

std::tuple<monotonic_clock::time_point, chrono::nanoseconds>
NoncausalTimestampFilter::GetReferenceTimestamp(
    monotonic_clock::time_point ta_base, double ta) const {
  DCHECK_GE(ta, 0.0);
  DCHECK_LT(ta, 1.0);
  std::tuple<monotonic_clock::time_point, chrono::nanoseconds>
      reference_timestamp = timestamp(0);
  if (ta_base >= std::get<0>(timestamp(timestamps_size() - 1u))) {
    reference_timestamp = timestamp(timestamps_size() - 1u);
  }

  return reference_timestamp;
}

chrono::nanoseconds NoncausalTimestampFilter::Offset(
    monotonic_clock::time_point ta) const {
  CHECK_GT(timestamps_size(), 0u);
  if (IsOutsideSamples(ta, 0.)) {
    // Special case when size = 1 or if we're asked to extrapolate to
    // times before or after we have data.
    auto reference_timestamp = GetReferenceTimestamp(ta, 0.);
    return NoncausalTimestampFilter::ExtrapolateOffset(reference_timestamp, ta);
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta);
  return NoncausalTimestampFilter::InterpolateOffset(points.first,
                                                     points.second, ta);
}

std::pair<chrono::nanoseconds, double> NoncausalTimestampFilter::Offset(
    monotonic_clock::time_point ta_base, double ta) const {
  CHECK_GT(timestamps_size(), 0u);
  if (IsOutsideSamples(ta_base, ta)) {
    // Special case size = 1 or ta_base before first timestamp or
    // after last timesteamp, so we need to extrapolate out
    auto reference_timestamp = GetReferenceTimestamp(ta_base, ta);
    return std::make_pair(NoncausalTimestampFilter::ExtrapolateOffset(
                              reference_timestamp, ta_base, ta),
                          NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
                              reference_timestamp, ta_base, ta));
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta_base, ta);
  // Return both the integer and double portion together to save a timestamp
  // lookup.
  return std::make_pair(NoncausalTimestampFilter::InterpolateOffset(
                            points.first, points.second, ta_base, ta),
                        NoncausalTimestampFilter::InterpolateOffsetRemainder(
                            points.first, points.second, ta_base, ta));
}

double NoncausalTimestampFilter::OffsetError(
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  const auto offset = Offset(ta_base, ta);

  // Compute the integer portion first, and the double portion second.  Subtract
  // the results of each.  This handles large offsets without losing precision.
  return static_cast<double>(((tb_base - ta_base) - offset.first).count()) +
         ((tb - ta) - offset.second);
}

double NoncausalTimestampFilter::Cost(aos::monotonic_clock::time_point ta_base,
                                      double ta,
                                      aos::monotonic_clock::time_point tb_base,
                                      double tb) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  // Squaring the error throws away half the digits.  The optimizer uses the
  // gradient heavily to compensate, so we don't need to care much about
  // computing this carefully.
  return std::pow(OffsetError(ta_base, ta, tb_base, tb), 2.0);
}

double NoncausalTimestampFilter::DCostDta(
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb) const {
  // As a reminder, our cost function is:
  //   (OffsetError(ta, tb))^2
  // ie
  //   ((tb - ta - Offset(ta))^2
  //
  // Assuming Offset(ta) = m * ta + ba (linear): this becomes
  //   ((tb - ta - (ma ta + ba))^2
  // ie
  //   ((tb - (1 + ma) ta - ba)^2
  //
  // d cost/dta =>
  //   2 * (tb - (1 + ma) ta - ba) * (-(1 + ma))
  //
  // We don't actually want to compute tb - (1 + ma) ta for numerical precision
  // reasons.  The important digits are small compared to the offset.  Given our
  // original cost above, this is equivalent to:
  //   - 2 * (tb - ta - Offset(ta)) * (1 + ma)
  //
  // We can compute this a lot more accurately.

  // Go find our timestamps for the interpolation.
  // Rather than lookup timestamps a number of times, look them up here and
  // inline the implementation of OffsetError.
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  if (IsOutsideSamples(ta_base, ta)) {
    double slope = kMaxVelocity();
    if (IsAfterSamples(ta_base, ta)) {
      // If we're past the last sample point, use a negative slope
      slope = -kMaxVelocity();
    }
    // This implements the high precision version of the above equation:
    //   - 2 * (tb - ta - Offset(ta)) * (1 + ma)
    //      = -2 * OffsetError(ta,tb) * (1 + ma)
    // Where for extrapolated data, we have to extrapolate for Offset(ta)
    // and use +/- kMaxVelocity() for ma
    // (slope is positive if timepoint is before our first sample, and
    //   negative if the point is after our last sample)
    return -2.0 * OffsetError(ta_base, ta, tb_base, tb) * (1.0 + slope);
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta_base, ta);

  const double m =
      static_cast<double>(
          (std::get<1>(points.second) - std::get<1>(points.first)).count()) /
      static_cast<double>(
          (std::get<0>(points.second) - std::get<0>(points.first)).count());
  // Subtract the integer offsets first and then handle the double remainder to
  // keep precision up.
  //
  //   (tb - ta - Offset(ta)) ->
  //      (tb_base - ta_base - OffsetBase + tb - ta - OffsetRemainder)
  // NOTE: We don't use OffsetError function here, in order to avoid
  // extra calls to FindTimestamps
  return -2.0 *
         (static_cast<double>((tb_base - ta_base -
                               NoncausalTimestampFilter::InterpolateOffset(
                                   points.first, points.second, ta_base, ta))
                                  .count()) +
          (tb - ta) -
          NoncausalTimestampFilter::InterpolateOffsetRemainder(
              points.first, points.second, ta_base, ta)) *
         (1.0 + m);
}

std::string NoncausalTimestampFilter::DebugDCostDta(
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb, size_t node_a,
    size_t node_b) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  if (IsOutsideSamples(ta_base, ta)) {
    auto reference_timestamp = GetReferenceTimestamp(ta_base, ta);
    double slope = kMaxVelocity();
    std::string note = "_";
    if (IsAfterSamples(ta_base, ta)) {
      slope = -kMaxVelocity();
      note = "^";
    }
    // d cost / dta ==>
    // - 2 * (tb - ta - (ta - ref) * ma - ref_offset) * (1 + ma)
    return absl::StrFormat(
        "-2. * (t%d - t%d - ((t%d - %d) * %f - %d.) * (1 + %f.)%s", node_b,
        node_a, node_a,
        std::get<0>(reference_timestamp).time_since_epoch().count(), slope,
        std::get<1>(reference_timestamp).count(), slope, note);
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta_base, ta);

  // As a reminder, our cost function is essentially:
  //   ((tb - ta - (ma ta + ba))^2
  // ie
  //   ((tb - (1 + ma) ta - ba)^2
  //
  // d cost/dta =>
  //   2 * (tb - (1 + ma) ta - ba) * (-(1 + ma))

  const int64_t rise =
      (std::get<1>(points.second) - std::get<1>(points.first)).count();
  const int64_t run =
      (std::get<0>(points.second) - std::get<0>(points.first)).count();

  if (rise == 0) {
    return absl::StrFormat("-2. * (t%d - t%d %c %d.)", node_b, node_a,
                           std::get<1>(points.first).count() >= 0 ? '-' : '+',
                           std::abs(std::get<1>(points.first).count()));
  } else {
    return absl::StrFormat(
        "-2. * (t%d - t%d - (t%d - %d.) * %d. / %d. - %d.) * (1 + %d. / "
        "%d.)",
        node_b, node_a, node_a,
        std::get<0>(points.first).time_since_epoch().count(), rise, run,
        std::get<1>(points.first).count(), rise, run);
  }
}

double NoncausalTimestampFilter::DCostDtb(
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb) const {
  // As a reminder, our cost function is:
  //   (OffsetError(ta, tb))^2
  //
  // d cost/dtb =>
  //   2 * OffsetError(ta, tb) * d/dtb OffsetError(ta, tb)
  //
  // OffsetError => (tb - (1 + ma) ta - ba), so
  //   d/dtb OffsetError(ta, tb) = 1
  //
  // d cost/dtb => 2 * OffsetError(ta, tb)
  NormalizeTimestamps(&ta_base, &ta);
  return 2.0 * OffsetError(ta_base, ta, tb_base, tb);
}

std::string NoncausalTimestampFilter::DebugDCostDtb(
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb, size_t node_a,
    size_t node_b) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  if (IsOutsideSamples(ta_base, ta)) {
    auto reference_timestamp = GetReferenceTimestamp(ta_base, ta);
    double slope = kMaxVelocity();
    std::string note = "_";
    if (IsAfterSamples(ta_base, ta)) {
      slope = -kMaxVelocity();
      note = "^";
    }
    // d cost / dtb ==> 2 * OffsetError(ta, tb) ==>
    // 2 * (tb - ta - (ta - ref) * ma - ref_offset)
    return absl::StrFormat(
        "2. * (t%d - t%d - ((t%d - %d) * %f + %d)%s", node_b, node_a, node_a,
        std::get<0>(reference_timestamp).time_since_epoch().count(), slope,
        std::get<1>(reference_timestamp).count(), note);
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta_base, ta);

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

std::string NoncausalTimestampFilter::DebugCost(
    aos::monotonic_clock::time_point ta_base, double ta,
    aos::monotonic_clock::time_point tb_base, double tb, size_t node_a,
    size_t node_b) const {
  NormalizeTimestamps(&ta_base, &ta);
  NormalizeTimestamps(&tb_base, &tb);

  if (IsOutsideSamples(ta_base, ta)) {
    auto reference_timestamp = GetReferenceTimestamp(ta_base, ta);
    double slope = kMaxVelocity();
    std::string note = "_";
    if (IsAfterSamples(ta_base, ta)) {
      slope = -kMaxVelocity();
      note = "^";
    }
    // Cost = OffsetError(ta, tb) ** 2 = (tb - ta - Offset(ta, tb)) ** 2 ==>
    // (tb - ta - ((ta - ref) * ma - ref_offset)
    return absl::StrFormat(
        "(t%d - t%d - (t%d - %d.) * %f - %d.)%s ** 2.", node_b, node_a, node_a,
        std::get<0>(reference_timestamp).time_since_epoch().count(), slope,
        std::get<1>(reference_timestamp).count(), note);
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta_base, ta);

  // As a reminder, our cost function is essentially:
  //   ((tb - ta - (ma ta + ba))^2
  // ie
  //   ((tb - (1 + ma) ta - ba)^2
  //
  // d cost/dta =>
  //   2 * ((tb - (1 + ma) ta - ba)

  const int64_t rise =
      (std::get<1>(points.second) - std::get<1>(points.first)).count();
  const int64_t run =
      (std::get<0>(points.second) - std::get<0>(points.first)).count();

  if (rise == 0) {
    return absl::StrFormat("(t%d - t%d %c %d.) ** 2.", node_b, node_a,
                           std::get<1>(points.first).count() < 0 ? '+' : '-',
                           std::abs(std::get<1>(points.first).count()));
  } else {
    return absl::StrFormat("(t%d - t%d - (t%d - %d.) * %d. / %d. - %d.) ** 2.",
                           node_b, node_a, node_a,
                           std::get<0>(points.first).time_since_epoch().count(),
                           rise, run, std::get<1>(points.first).count());
  }
}

std::string NoncausalTimestampFilter::NodeNames() const {
  return absl::StrCat(node_a_->name()->string_view(), " -> ",
                      node_b_->name()->string_view());
}

bool NoncausalTimestampFilter::ValidateSolution(
    aos::monotonic_clock::time_point ta,
    aos::monotonic_clock::time_point tb) const {
  CHECK_GT(timestamps_size(), 0u);
  if (ta < std::get<0>(timestamp(0)) && has_popped_) {
    LOG(ERROR) << NodeNames() << " O(" << ta
               << ") is before the start and we have forgotten the answer.";
    return false;
  }
  if (IsOutsideSamples(ta, 0.)) {
    // Special case size = 1 or ta_base before first timestamp or
    // after last timestamp, so we need to extrapolate out
    auto reference_timestamp = GetReferenceTimestamp(ta, 0.);

    // Special case size = 1 or ta before first timestamp, so we extrapolate
    const chrono::nanoseconds offset =
        NoncausalTimestampFilter::ExtrapolateOffset(reference_timestamp, ta);
    if (offset + ta > tb) {
      LOG(ERROR) << NodeNames() << " " << TimeString(ta, offset)
                 << " > solution time " << tb;
      return false;
    }
    return true;
  }

  std::pair<std::tuple<monotonic_clock::time_point, chrono::nanoseconds>,
            std::tuple<monotonic_clock::time_point, chrono::nanoseconds>>
      points = FindTimestamps(ta);
  const chrono::nanoseconds offset =
      NoncausalTimestampFilter::InterpolateOffset(points.first, points.second,
                                                  ta);
  if (offset + ta > tb) {
    LOG(ERROR) << NodeNames() << " " << TimeString(ta, offset)
               << " > solution time " << tb;
    LOG(ERROR) << "Bracketing times are " << TimeString(points.first) << " and "
               << TimeString(points.second);
    return false;
  }
  return true;
}

void NoncausalTimestampFilter::Sample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  if (samples_fp_) {
    saved_samples_.emplace_back(std::make_pair(monotonic_now, sample_ns));
    if (first_time_ != aos::monotonic_clock::min_time) {
      FlushSavedSamples();
    }
  }

  // The first sample is easy.  Just do it!
  if (timestamps_.size() == 0) {
    VLOG(1) << NodeNames() << " Initial sample of "
            << TimeString(monotonic_now, sample_ns);
    timestamps_.emplace_back(std::make_tuple(monotonic_now, sample_ns));
    CHECK(!fully_frozen_)
        << ": " << NodeNames()
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
      << ": " << NodeNames() << " Tried to insert " << monotonic_now
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
    VLOG(1) << NodeNames() << " Duplicate sample of O(" << monotonic_now
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
    if (doffset < -dt * kMaxVelocity()) {
      VLOG(1) << std::setprecision(1) << std::fixed << NodeNames()
              << " Rejected sample of " << TimeString(monotonic_now, sample_ns)
              << " because " << doffset.count() << " < "
              << (-dt * kMaxVelocity()).count() << " len "
              << timestamps_.size();
      return;
    }

    // Be overly conservative here.  It either won't make a difference, or
    // will give us an error with an actual useful time difference.
    CHECK(!fully_frozen_)
        << ": " << NodeNames()
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
    while (dt * kMaxVelocity() < doffset && timestamps_.size() > 1u) {
      CHECK(!frozen(std::get<0>(back)))
          << ": " << NodeNames() << " Can't pop an already frozen sample "
          << TimeString(back) << " while inserting "
          << TimeString(monotonic_now, sample_ns) << ", "
          << chrono::duration<double>(monotonic_now - std::get<0>(back)).count()
          << " seconds in the past.  Increase --time_estimation_buffer_seconds "
             "to greater than "
          << chrono::duration<double>(monotonic_now - std::get<0>(back))
                 .count();
      VLOG(1) << NodeNames()
              << " Removing now invalid sample during back propegation of "
              << TimeString(back);
      timestamps_.pop_back();

      back = timestamps_.back();
      dt = monotonic_now - std::get<0>(back);
      doffset = sample_ns - std::get<1>(back);
    }

    VLOG(1) << NodeNames() << " Added sample of "
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
        VLOG(1) << NodeNames() << " Redundant timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(timestamps_.front())
                << " is at the same time and a better solution.";
        return;
      }
    }

    VLOG(1) << NodeNames() << " Added sample at beginning "
            << TimeString(monotonic_now, sample_ns);
    timestamps_.insert(it, std::make_tuple(monotonic_now, sample_ns));

    while (true) {
      // First point was too positive, so we need to remove points after it
      // until we are valid.
      auto second = timestamps_.begin() + 1;
      if (second != timestamps_.end()) {
        const chrono::nanoseconds dt = std::get<0>(*second) - monotonic_now;
        const chrono::nanoseconds doffset = std::get<1>(*second) - sample_ns;

        if (doffset < -dt * kMaxVelocity()) {
          VLOG(1) << NodeNames() << " Removing redundant sample of "
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

          if (doffset > dt * kMaxVelocity()) {
            VLOG(1) << NodeNames() << " Removing invalid sample of "
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
    VLOG(1) << NodeNames() << " Found the next time " << std::get<0>(*(it - 1))
            << " < " << monotonic_now << " < " << std::get<0>(*it);

    {
      chrono::nanoseconds prior_dt = monotonic_now - std::get<0>(*(it - 1));
      chrono::nanoseconds prior_doffset = sample_ns - std::get<1>(*(it - 1));
      chrono::nanoseconds next_dt = std::get<0>(*it) - monotonic_now;
      chrono::nanoseconds next_doffset = std::get<1>(*it) - sample_ns;

      // If we are worse than either the previous or next point, discard.
      if (prior_doffset < -prior_dt * kMaxVelocity()) {
        VLOG(1) << NodeNames() << " Ignoring timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(*(it - 1))
                << " is before and the slope would be too negative.";
        return;
      }
      if (next_doffset > next_dt * kMaxVelocity()) {
        VLOG(1) << NodeNames() << " Ignoring timestamp "
                << TimeString(monotonic_now, sample_ns) << " because "
                << TimeString(*it)
                << " is following and the slope would be too positive.";
        return;
      }
    }

    // Now, insert and start propagating forwards and backwards anything we've
    // made invalid.  Do this simultaneously so we keep discovering anything
    // new.
    auto middle_it =
        timestamps_.insert(it, std::make_tuple(monotonic_now, sample_ns));
    VLOG(1) << NodeNames() << " Inserted " << TimeString(*middle_it);

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

        if (next_doffset < -next_dt * kMaxVelocity()) {
          VLOG(1) << NodeNames()
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

        if (prior_doffset > prior_dt * kMaxVelocity()) {
          CHECK(!frozen(std::get<0>(*prior_it)))
              << ": " << NodeNames()
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

bool NoncausalTimestampFilter::Pop(aos::monotonic_clock::time_point time) {
  VLOG(1) << NodeNames() << " Pop(" << time << ")";
  bool removed = false;
  // When the timestamp which is the end of the line is popped, we want to
  // drop it off the list.  Hence the >=
  while (timestamps_.size() >= 2 && time >= std::get<0>(timestamps_[1])) {
    PopFront();
    removed = true;
  }
  return removed;
}

void NoncausalTimestampFilter::Debug() {
  size_t count = 0;
  for (std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
           timestamp : timestamps_) {
    LOG(INFO) << NodeNames() << " "
              << TimeString(std::get<0>(timestamp), std::get<1>(timestamp))
              << " frozen? " << frozen(std::get<0>(timestamp)) << " consumed? "
              << (count < next_to_consume_);
    ++count;
  }
}

monotonic_clock::time_point NoncausalTimestampFilter::unobserved_line_end()
    const {
  if (has_unobserved_line()) {
    return std::get<0>(timestamp(next_to_consume_ + 1));
  }
  return monotonic_clock::min_time;
}

monotonic_clock::time_point
NoncausalTimestampFilter::unobserved_line_remote_end() const {
  if (has_unobserved_line()) {
    const std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> t =
        timestamp(next_to_consume_ + 1);
    return std::get<0>(t) + std::get<1>(t);
  }
  return monotonic_clock::min_time;
}

bool NoncausalTimestampFilter::has_unobserved_line() const {
  return next_to_consume_ + 1 < timestamps_.size();
}

std::optional<std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
NoncausalTimestampFilter::Observe() const {
  if (timestamps_.empty() || next_to_consume_ >= timestamps_.size()) {
    return std::nullopt;
  }
  VLOG(1) << NodeNames() << " Observed sample of "
          << TimeString(timestamp(next_to_consume_));
  return timestamp(next_to_consume_);
}

std::optional<std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>>
NoncausalTimestampFilter::Consume() {
  if (timestamps_.empty() || next_to_consume_ >= timestamps_.size()) {
    return std::nullopt;
  }

  auto result = timestamp(next_to_consume_);
  VLOG(1) << NodeNames() << " Consumed sample of " << TimeString(result);
  ++next_to_consume_;
  return result;
}

void NoncausalTimestampFilter::FreezeUntil(
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
    VLOG(1) << NodeNames() << " fully_frozen_, no timestamps.";
    fully_frozen_ = true;
  } else if (node_monotonic_now > std::get<0>(timestamps_.back())) {
    // We've been asked to freeze past the last point.  It isn't safe to add any
    // more points or we will change this region.
    VLOG(1) << NodeNames() << " fully_frozen_, after the end.";
    fully_frozen_ = true;
  } else {
    LOG(FATAL) << "How did we get here?";
  }
}

void NoncausalTimestampFilter::FreezeUntilRemote(
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
    VLOG(1) << NodeNames() << " fully_frozen_, no timestamps.";
    fully_frozen_ = true;
  } else if (remote_monotonic_now > std::get<0>(timestamps_.back()) +
                                        std::get<1>(timestamps_.back())) {
    // We've been asked to freeze past the last point.  It isn't safe to add any
    // more points or we will change this region.
    VLOG(1) << NodeNames() << " fully_frozen_, after the end.";
    fully_frozen_ = true;
  } else {
    LOG(FATAL) << "How did we get here?";
  }
}

void NoncausalTimestampFilter::SetFirstTime(
    aos::monotonic_clock::time_point time) {
  first_time_ = time;
  if (fp_) {
    fp_ = freopen(NULL, "wb", fp_);
    PrintNoncausalTimestampFilterHeader(fp_);
  }
  if (samples_fp_) {
    samples_fp_ = freopen(NULL, "wb", samples_fp_);
    PrintNoncausalTimestampFilterSamplesHeader(samples_fp_);
    FlushSavedSamples();
  }
}

void NoncausalTimestampFilter::SetCsvFileName(std::string_view name) {
  fp_ = fopen(absl::StrCat(name, ".csv").c_str(), "w");
  samples_fp_ = fopen(absl::StrCat(name, "_samples.csv").c_str(), "w");
  PrintNoncausalTimestampFilterHeader(fp_);
  PrintNoncausalTimestampFilterSamplesHeader(samples_fp_);
}

void NoncausalTimestampFilter::PopFront() {
  VLOG(1) << NodeNames() << " Popped sample of " << TimeString(timestamp(0));
  MaybeWriteTimestamp(timestamp(0));

  // If we drop data, we shouldn't add anything before that point.
  frozen_time_ = std::max(frozen_time_, std::get<0>(timestamp(0)));
  timestamps_.pop_front();
  has_popped_ = true;
  if (next_to_consume_ > 0u) {
    next_to_consume_--;
  }
}

void NoncausalTimestampFilter::MaybeWriteTimestamp(
    std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
        timestamp) {
  if (fp_ && first_time_ != aos::monotonic_clock::min_time) {
    fprintf(fp_, "%.9f, %.9f, %.9f\n",
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::get<0>(timestamp) - first_time_)
                .count(),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::get<0>(timestamp).time_since_epoch())
                .count(),
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::get<1>(timestamp))
                .count());
    fflush(fp_);
  }
}

void NoncausalOffsetEstimator::Sample(
    const Node *node, aos::monotonic_clock::time_point node_delivered_time,
    aos::monotonic_clock::time_point other_node_sent_time) {
  VLOG(1) << "Sample delivered         " << node_delivered_time << " sent "
          << other_node_sent_time << " " << node->name()->string_view()
          << " -> "
          << ((node == node_a_) ? node_b_ : node_a_)->name()->string_view();
  if (node == node_a_) {
    a_.Sample(node_delivered_time, other_node_sent_time - node_delivered_time);
  } else if (node == node_b_) {
    b_.Sample(node_delivered_time, other_node_sent_time - node_delivered_time);
  } else {
    LOG(FATAL) << "Unknown node " << node->name()->string_view();
  }
}

void NoncausalOffsetEstimator::ReverseSample(
    const Node *node, aos::monotonic_clock::time_point node_sent_time,
    aos::monotonic_clock::time_point other_node_delivered_time) {
  VLOG(1) << "Reverse sample delivered " << other_node_delivered_time
          << " sent " << node_sent_time << " "
          << ((node == node_a_) ? node_b_ : node_a_)->name()->string_view()
          << " -> " << node->name()->string_view();
  if (node == node_a_) {
    b_.Sample(other_node_delivered_time,
              node_sent_time - other_node_delivered_time);
  } else if (node == node_b_) {
    a_.Sample(other_node_delivered_time,
              node_sent_time - other_node_delivered_time);
  } else {
    LOG(FATAL) << "Unknown node " << node->name()->string_view();
  }
}

bool NoncausalOffsetEstimator::Pop(
    const Node *node, aos::monotonic_clock::time_point node_monotonic_now) {
  if (node == node_a_) {
    if (a_.Pop(node_monotonic_now)) {
      VLOG(1) << "Popping forward sample to " << node_a_->name()->string_view()
              << " from " << node_b_->name()->string_view() << " at "
              << node_monotonic_now;
      return true;
    }
  } else if (node == node_b_) {
    if (b_.Pop(node_monotonic_now)) {
      VLOG(1) << "Popping reverse sample to " << node_b_->name()->string_view()
              << " from " << node_a_->name()->string_view() << " at "
              << node_monotonic_now;
      return true;
    }
  } else {
    LOG(FATAL) << "Unknown node " << node->name()->string_view();
  }
  return false;
}

}  // namespace message_bridge
}  // namespace aos
