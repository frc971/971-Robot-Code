#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/timing_report_dump_lib.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "The path to the config to use.");
ABSL_FLAG(std::string, application, "",
          "Application filter to use. Empty for no filter.");
ABSL_FLAG(bool, stream, true,
          "Stream out all the timing reports that we receive.");
ABSL_FLAG(bool, accumulate, true,
          "Display accumulation of all timing reports that we've seen when "
          "the process is terminated.");

namespace aos {
int Main() {
  aos::FlatbufferVector<aos::Configuration> config(
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config)));
  ShmEventLoop event_loop(&config.message());
  TimingReportDump dumper(&event_loop,
                          absl::GetFlag(FLAGS_accumulate)
                              ? TimingReportDump::AccumulateStatistics::kYes
                              : TimingReportDump::AccumulateStatistics::kNo,
                          absl::GetFlag(FLAGS_stream)
                              ? TimingReportDump::StreamResults::kYes
                              : TimingReportDump::StreamResults::kNo);
  if (!absl::GetFlag(FLAGS_application).empty()) {
    dumper.ApplicationFilter(absl::GetFlag(FLAGS_application));
  }
  event_loop.Run();
  return EXIT_SUCCESS;
}
}  // namespace aos

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main();
}
