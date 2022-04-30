#include "aos/events/timing_report_dump_lib.h"

#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/events/logging/log_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(application, "",
              "Application filter to use. Empty for no filter.");
DEFINE_bool(stream, false, "Stream out all the timing reports in the log.");
DEFINE_bool(accumulate, true,
            "Display accumulation of all timing reports at end of log.");

namespace aos {
struct DumperState {
  std::unique_ptr<EventLoop> event_loop;
  std::unique_ptr<TimingReportDump> dumper;
};
int Main(int argc, char *argv[]) {
  if (argc < 2) {
    LOG(ERROR) << "Expected at least 1 logfile as an argument";
    return 1;
  }
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  reader.Register();
  {
    std::vector<DumperState> dumpers;
    for (const aos::Node *node : aos::configuration::GetNodes(
             reader.event_loop_factory()->configuration())) {
      std::unique_ptr<aos::EventLoop> event_loop =
          reader.event_loop_factory()->MakeEventLoop("timing_reports", node);
      event_loop->SkipTimingReport();
      event_loop->SkipAosLog();
      std::unique_ptr<TimingReportDump> dumper =
          std::make_unique<TimingReportDump>(
              event_loop.get(),
              FLAGS_accumulate ? TimingReportDump::AccumulateStatistics::kYes
                               : TimingReportDump::AccumulateStatistics::kNo,
              FLAGS_stream ? TimingReportDump::StreamResults::kYes
                           : TimingReportDump::StreamResults::kNo);
      if (!FLAGS_application.empty()) {
        dumper->ApplicationFilter(FLAGS_application);
      }
      dumpers.push_back({std::move(event_loop), std::move(dumper)});
    }
    reader.event_loop_factory()->Run();
  }
  reader.Deregister();
  return EXIT_SUCCESS;
}
}  // namespace aos

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
