#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/shm_event_loop.h"
#include "aos/logging/log_namer.h"
#include "aos/init.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "config.json", "Config file to use.");

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage(
      "This program provides a simple logger binary that logs all SHMEM data "
      "directly to a file specified at the command line. It does not manage "
      "filenames, so it will just crash if you attempt to overwrite an "
      "existing file, and the user must specify the logfile manually at the "
      "command line.");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  std::unique_ptr<aos::logger::DetachedBufferWriter> writer;
  std::unique_ptr<aos::logger::LogNamer> log_namer;
  if (event_loop.node() == nullptr) {
    writer = std::make_unique<aos::logger::DetachedBufferWriter>(
        aos::logging::GetLogName("fbs_log"));

    log_namer = std::make_unique<aos::logger::LocalLogNamer>(writer.get(),
                                                             event_loop.node());
  } else {
    log_namer = std::make_unique<aos::logger::MultiNodeLogNamer>(
        aos::logging::GetLogName("fbs_log"), event_loop.configuration(),
        event_loop.node());
  }

  aos::logger::Logger logger(std::move(log_namer), &event_loop,
                             std::chrono::milliseconds(100));

  event_loop.Run();

  aos::Cleanup();
  return 0;
}
