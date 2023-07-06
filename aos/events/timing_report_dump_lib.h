#ifndef AOS_EVENTS_TIMING_REPORT_DUMP_LIB_H_
#define AOS_EVENTS_TIMING_REPORT_DUMP_LIB_H_
#include <map>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"
#include "aos/json_to_flatbuffer.h"

namespace aos {
// A class to handle printing timing report statistics in a useful format on the
// command line. Main features:
// * Correlates channel indices to channel names/types.
// * Formats timing reports in a more readable manner than pure JSON.
// * Can filter on application name.
// * Can accumulate all the timing reports for an application over time and
//   produce summary statistics.
class TimingReportDump {
 public:
  enum class AccumulateStatistics { kYes, kNo };
  enum class StreamResults { kYes, kNo };
  TimingReportDump(aos::EventLoop *event_loop, AccumulateStatistics accumulate,
                   StreamResults stream);
  // The destructor handles the final printout of accumulated statistics (if
  // requested), which for log reading should happen after the log has been
  // fully replayed and for live systems will happen the user Ctrl-C's.
  ~TimingReportDump();

  // Filter to use for application name. Currently requires that the provided
  // name exactly match the name of the application in question.
  void ApplicationFilter(std::string_view name) { name_filter_ = name; }

 private:
  void HandleTimingReport(const timing::Report &report);
  const Channel *GetChannel(int index);
  void PrintTimers(
      std::ostream *os, std::string_view name,
      const flatbuffers::Vector<flatbuffers::Offset<timing::Timer>> &timers);
  void PrintWatchers(
      std::ostream *os,
      const flatbuffers::Vector<flatbuffers::Offset<timing::Watcher>>
          &watchers);
  void PrintSenders(
      std::ostream *os,
      const flatbuffers::Vector<flatbuffers::Offset<timing::Sender>> &senders);
  void PrintFetchers(
      std::ostream *os,
      const flatbuffers::Vector<flatbuffers::Offset<timing::Fetcher>>
          &fetchers);
  void PrintReport(const timing::Report &report);
  void AccumulateReport(const timing::Report &report);

  aos::EventLoop *event_loop_;
  AccumulateStatistics accumulate_;
  StreamResults stream_;
  std::optional<std::string> name_filter_;
  // Key is pair of <process id, application name>, since neither is a unique
  // identifier across time.
  std::map<std::pair<pid_t, std::string>, timing::ReportT>
      accumulated_statistics_;
};
}  // namespace aos
#endif  // AOS_EVENTS_TIMING_REPORT_DUMP_LIB_H_
