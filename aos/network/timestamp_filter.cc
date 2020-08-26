#include "aos/network/timestamp_filter.h"

#include <chrono>
#include <iomanip>
#include <tuple>

#include "absl/strings/str_cat.h"
#include "aos/time/time.h"
#include "third_party/gmp/gmpxx.h"

namespace aos {
namespace message_bridge {
namespace {

namespace chrono = std::chrono;

void ClippedAverageFilterPrintHeader(FILE *fp) {
  fprintf(fp,
          "# time_since_start, sample_ns, filtered_offset, offset, "
          "velocity, filtered_velocity, velocity_contribution, "
          "sample_contribution, time_contribution\n");
}

void PrintNoncausalTimestampFilterHeader(FILE *fp) {
  fprintf(fp,
          "# time_since_start, sample_ns, filtered_offset, offset, "
          "velocity, filtered_velocity, velocity_contribution, "
          "sample_contribution, time_contribution\n");
}

void PrintNoncausalTimestampFilterSamplesHeader(FILE *fp) {
  fprintf(fp, "# time_since_start, sample_ns, offset\n");
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
      << ": Being asked to filter backwards in time!";
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
  VLOG(1) << "  Max(fwd) " << hard_max << " min(rev) " << hard_min;
  // We don't want to clip the offset to the hard min/max.  We really want to
  // keep it within a band around the middle.  ratio of 0.3 means stay within
  // +- 0.15 of the middle of the hard min and max.
  constexpr double kBand = 0.3;
  const double max = average + kBand / 2.0 * (hard_max - hard_min);
  const double min = average - kBand / 2.0 * (hard_max - hard_min);

  // Update regardless for the first sample from both the min and max.
  if (*last_time == aos::monotonic_clock::min_time) {
    VLOG(1) << "  No last time " << average;
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

    VLOG(1) << "  last time " << offset_;
  }
  *last_time = monotonic_now;

  if (sample_pointer_ != nullptr) {
    // TODO(austin): Probably shouldn't do the update if we don't have fwd and
    // reverse samples.
    if (!MissingSamples()) {
      *sample_pointer_ = offset_;
      VLOG(1) << "Updating sample to " << offset_;
    } else {
      VLOG(1) << "Don't have both samples.";
      if (last_fwd_time_ == aos::monotonic_clock::min_time) {
        VLOG(1) << " Missing forward";
      }
      if (last_rev_time_ == aos::monotonic_clock::min_time) {
        VLOG(1) << " Missing reverse";
      }
    }
  }
}

Line Line::Fit(
    const std::tuple<monotonic_clock::time_point, chrono::nanoseconds> a,
    const std::tuple<monotonic_clock::time_point, chrono::nanoseconds> b) {
  mpq_class slope = FromInt64((std::get<1>(b) - std::get<1>(a)).count()) /
                    FromInt64((std::get<0>(b) - std::get<0>(a)).count());
  slope.canonicalize();
  mpq_class offset =
      FromInt64(std::get<1>(a).count()) -
      FromInt64(std::get<0>(a).time_since_epoch().count()) * slope;
  offset.canonicalize();
  Line f(offset, slope);
  return f;
}

Line AverageFits(Line fa, Line fb) {
  // tb = Oa(ta) + ta
  // ta = Ob(tb) + tb
  // tb - ta = Oa(ta)
  // tb - ta = -Ob(tb)
  // Oa(ta) = ma * ta + ba
  // Ob(tb) = mb * tb + bb
  //
  // ta + O(ta, tb) = tb
  // tb - ta = O(ta, tb)
  // O(ta, tb) = (Oa(ta) - Ob(tb)) / 2.0
  // ta + (ma * ta + ba - mb * tb - bb) / 2 = tb
  // (2 + ma) / 2 * ta + (ba - bb) / 2 = (2 + mb) / 2 * tb
  // (2 + ma) * ta + (ba - bb) = (2 + mb) * tb
  // tb = (2 + ma) / (2 + mb) * ta + (ba - bb) / (2 + mb)
  // ta = (2 + mb) / (2 + ma) * tb + (bb - ba) / (2 + ma)
  //
  // ta - tb = (mb - ma) / (2 + ma) * tb + (bb - ba) / (2 + ma)
  // mb = (mb - ma) / (2 + ma)
  // bb = (bb - ba) / (2 + ma)
  //
  // tb - ta = (ma - mb) / (2 + mb) * tb + (ba - bb) / (2 + mb)
  // ma = (ma - mb) / (2 + mb)
  // ba = (ba - bb) / (2 + mb)
  //
  // O(ta) = ma * ta + ba
  // tb = O(ta) + ta
  // ta = O(tb) + tb

  mpq_class m =
      (fa.mpq_slope() - fb.mpq_slope()) / (mpq_class(2) + fb.mpq_slope());
  m.canonicalize();

  mpq_class b =
      (fa.mpq_offset() - fb.mpq_offset()) / (mpq_class(2) + fb.mpq_slope());
  b.canonicalize();

  Line f(b, m);
  return f;
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

Line NoncausalTimestampFilter::FitLine() {
  DCHECK_GE(timestamps_.size(), 1u);
  if (timestamps_.size() == 1) {
    Line fit(std::get<1>(timestamps_[0]), 0.0);
    return fit;
  } else {
    return Line::Fit(timestamps_[0], timestamps_[1]);
  }
}

bool NoncausalTimestampFilter::Sample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  if (samples_fp_ && first_time_ != aos::monotonic_clock::min_time) {
    fprintf(samples_fp_, "%.9f, %.9f\n",
            chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                            first_time_)
                .count(),
            chrono::duration_cast<chrono::duration<double>>(sample_ns).count());
  }

  // The first sample is easy.  Just do it!
  if (timestamps_.size() == 0) {
    timestamps_.emplace_back(std::make_pair(monotonic_now, sample_ns));
    return true;
  } else {
    // Future samples get quite a bit harder.  We want the line to track the
    // highest point without volating the slope constraint.
    std::tuple<aos::monotonic_clock::time_point, chrono::nanoseconds> back =
        timestamps_.back();

    aos::monotonic_clock::duration dt = monotonic_now - std::get<0>(back);
    aos::monotonic_clock::duration doffset = sample_ns - std::get<1>(back);

    // If the point is higher than the max negative slope, the slope will either
    // adhere to our constraint, or will be too positive.  If it is too
    // positive, we need to back propagate and remove offending points which
    // were too low rather than reject this new point.  We never want a point to
    // be higher than the line.
    if (-dt * kMaxVelocity() <= doffset) {
      // Back propagate the max velocity and remove any elements violating the
      // velocity constraint.
      while (dt * kMaxVelocity() < doffset && timestamps_.size() > 1u) {
        timestamps_.pop_back();

        back = timestamps_.back();
        dt = monotonic_now - std::get<0>(back);
        doffset = sample_ns - std::get<1>(back);
      }

      // TODO(austin): Refuse to modify the 0th element after we have used it.
      timestamps_.emplace_back(std::make_pair(monotonic_now, sample_ns));

      // If we are early in the log file, the filter hasn't had time to get
      // started.  We might only have 2 samples, and the first sample was
      // incredibly delayed, violating our velocity constraint.  In that case,
      // modify the first sample (rather than remove it) to retain the knowledge
      // of the velocity, but adhere to the constraints.
      if (dt * kMaxVelocity() < doffset) {
        CHECK_EQ(timestamps_.size(), 2u);
        const aos::monotonic_clock::duration adjusted_initial_time =
            sample_ns - aos::monotonic_clock::duration(
                            static_cast<aos::monotonic_clock::duration::rep>(
                                dt.count() * kMaxVelocity()));

        VLOG(1) << csv_file_name_ << " slope " << std::setprecision(20)
                << FitLine().slope() << " offset " << FitLine().offset().count()
                << " a [(" << std::get<0>(timestamps()[0]) << " -> "
                << std::get<1>(timestamps()[0]).count() << "ns), ("
                << std::get<0>(timestamps()[1]) << " -> "
                << std::get<1>(timestamps()[1]).count()
                << "ns) => {dt: " << std::fixed << std::setprecision(6)
                << chrono::duration<double, std::milli>(
                       std::get<0>(timestamps()[1]) -
                       std::get<0>(timestamps()[0]))
                       .count()
                << "ms, do: " << std::fixed << std::setprecision(6)
                << chrono::duration<double, std::milli>(
                       std::get<1>(timestamps()[1]) -
                       std::get<1>(timestamps()[0]))
                       .count()
                << "ms}]";
        VLOG(1) << "Back is out of range, clipping from "
                << std::get<1>(timestamps_[0]).count() << " to "
                << adjusted_initial_time.count();

        std::get<1>(timestamps_[0]) = adjusted_initial_time;
      }
      if (timestamps_.size() == 2) {
        return true;
      }
    }
    return false;
  }
}

bool NoncausalTimestampFilter::Pop(
    aos::monotonic_clock::time_point time) {
  bool removed = false;
  // When the timestamp which is the end of the line is popped, we want to
  // drop it off the list.  Hence the >=
  while (timestamps_.size() >= 2 && time >= std::get<0>(timestamps_[1])) {
    PopFront();
    removed = true;
  }
  return removed;
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
  }
}

void NoncausalTimestampFilter::SetCsvFileName(std::string_view name) {
  csv_file_name_ = name;
  fp_ = fopen(absl::StrCat(csv_file_name_, ".csv").c_str(), "w");
  samples_fp_ =
      fopen(absl::StrCat(csv_file_name_, "_samples.csv").c_str(), "w");
  PrintNoncausalTimestampFilterHeader(fp_);
  PrintNoncausalTimestampFilterSamplesHeader(samples_fp_);
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
  }
}

}  // namespace message_bridge
}  // namespace aos
