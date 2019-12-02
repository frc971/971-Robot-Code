#include "aos/events/timing_statistics.h"

#include "aos/events/event_loop_generated.h"
#include "glog/logging.h"

namespace aos {
namespace internal {

void RawFetcherTiming::set_timing_report(timing::Fetcher *new_fetcher) {
  CHECK_NOTNULL(new_fetcher);
  fetcher = new_fetcher;
  latency.set_statistic(fetcher->mutable_latency());
}

void RawFetcherTiming::ResetTimingReport() {
  latency.Reset();
  fetcher->mutate_count(0);
}

void RawSenderTiming::set_timing_report(timing::Sender *new_sender) {
  CHECK_NOTNULL(new_sender);
  sender = new_sender;
  size.set_statistic(sender->mutable_size());
}

void RawSenderTiming::ResetTimingReport() {
  size.Reset();
  sender->mutate_count(0);
}

void TimerTiming::set_timing_report(timing::Timer *new_timer) {
  CHECK_NOTNULL(new_timer);
  timer = new_timer;
  wakeup_latency.set_statistic(timer->mutable_wakeup_latency());
  handler_time.set_statistic(timer->mutable_handler_time());
}

void TimerTiming::ResetTimingReport() {
  wakeup_latency.Reset();
  handler_time.Reset();
  timer->mutate_count(0);
}

}  // namespace internal
}  // namespace aos
