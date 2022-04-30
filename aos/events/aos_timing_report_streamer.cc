#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/timing_report_dump_lib.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_string(config, "/app/aos_config.json", "The path to the config to use.");
DEFINE_string(application, "",
              "Application filter to use. Empty for no filter.");
DEFINE_bool(stream, true, "Stream out all the timing reports that we receive.");
DEFINE_bool(accumulate, true,
            "Display accumulation of all timing reports that we've seen when "
            "the process is terminated.");

namespace aos {
int Main() {
  aos::FlatbufferVector<aos::Configuration> config(
      aos::configuration::ReadConfig(FLAGS_config));
  ShmEventLoop event_loop(&config.message());
  TimingReportDump dumper(&event_loop,
                          FLAGS_accumulate
                              ? TimingReportDump::AccumulateStatistics::kYes
                              : TimingReportDump::AccumulateStatistics::kNo,
                          FLAGS_stream ? TimingReportDump::StreamResults::kYes
                                       : TimingReportDump::StreamResults::kNo);
  if (!FLAGS_application.empty()) {
    dumper.ApplicationFilter(FLAGS_application);
  }
  event_loop.Run();
  return EXIT_SUCCESS;
}
}  // namespace aos

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main();
}
