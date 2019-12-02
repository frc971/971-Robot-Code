#ifndef AOS_EVENTS_TIMING_STATISTICS_H_
#define AOS_EVENTS_TIMING_STATISTICS_H_

#include <cmath>

#include "aos/events/event_loop_generated.h"

namespace aos {
namespace internal {

// Class to compute statistics for the timing report.
class TimingStatistic {
 public:
  TimingStatistic() {}

  // Sets the flatbuffer to mutate.
  void set_statistic(timing::Statistic *statistic) { statistic_ = statistic; }

  // Adds a sample to the statistic.
  void Add(float sample) {
    ++count_;
    if (count_ == 1) {
      statistic_->mutate_average(sample);
      statistic_->mutate_min(sample);
      statistic_->mutate_max(sample);
      statistic_->mutate_standard_deviation(0.0);
    } else {
      // https://en.wikipedia.org/wiki/Standard_deviation#Rapid_calculation_methods
      const float prior_average = statistic_->average();
      const float average = prior_average + (sample - prior_average) / count_;
      statistic_->mutate_average(average);
      statistic_->mutate_max(std::max(statistic_->max(), sample));
      statistic_->mutate_min(std::min(statistic_->min(), sample));

      Q_ = Q_ + (sample - prior_average) * (sample - average);
      statistic_->mutate_standard_deviation(std::sqrt(Q_ / (count_ - 1)));
    }
  }

  // Clears any accumulated statistics.
  void Reset() {
    statistic_->mutate_average(std::numeric_limits<float>::quiet_NaN());
    statistic_->mutate_min(std::numeric_limits<float>::quiet_NaN());
    statistic_->mutate_max(std::numeric_limits<float>::quiet_NaN());

    statistic_->mutate_standard_deviation(
        std::numeric_limits<float>::quiet_NaN());
    Q_ = 0;
    count_ = 0;
  }

 private:
  timing::Statistic *statistic_ = nullptr;
  // Number of samples accumulated.
  size_t count_ = 0;
  // State Q from wikipedia.
  float Q_ = 0.0;
};

// Class to hold timing information for a raw fetcher.
struct RawFetcherTiming {
  RawFetcherTiming(int new_channel_index) : channel_index(new_channel_index) {}

  void set_timing_report(timing::Fetcher *fetcher);
  void ResetTimingReport();

  const int channel_index;
  timing::Fetcher *fetcher = nullptr;
  internal::TimingStatistic latency;
};

// Class to hold timing information for a raw sender.
struct RawSenderTiming {
  RawSenderTiming(int new_channel_index) : channel_index(new_channel_index) {}

  void set_timing_report(timing::Sender *sender);
  void ResetTimingReport();

  const int channel_index;
  timing::Sender *sender = nullptr;
  internal::TimingStatistic size;
};

// Class to hold timing information for timers.
struct TimerTiming {
  void set_timing_report(timing::Timer *timer);
  void ResetTimingReport();

  internal::TimingStatistic wakeup_latency;
  internal::TimingStatistic handler_time;
  timing::Timer *timer = nullptr;
};

}  // namespace internal
}  // namespace aos

#endif  // AOS_EVENTS_TIMING_STATISTICS_H_
