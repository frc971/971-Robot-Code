#include <sys/resource.h>
#include <sys/time.h>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/logging/snappy_encoder.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/log_namer.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Config file to use.");

ABSL_FLAG(double, rotate_every, 30.0,
          "If set, rotate the logger after this many seconds");

ABSL_DECLARE_FLAG(int32_t, flush_size);

int main(int argc, char *argv[]) {
  absl::SetProgramUsageMessage(
      "This program provides a simple logger binary that logs all SHMEM data "
      "directly to a file specified at the command line. It does not manage "
      "filenames, so it will just crash if you attempt to overwrite an "
      "existing file, and the user must specify the logfile manually at the "
      "command line.");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  auto log_namer = std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
      &event_loop,
      std::make_unique<aos::logger::RenamableFileBackend>(
          absl::StrCat(aos::logging::GetLogName("localizer_log"), "/"),
          /*O_DIRECT*/ true));

  log_namer->set_extension(aos::logger::SnappyDecoder::kExtension);
  log_namer->set_encoder_factory([](size_t max_message_size) {
    return std::make_unique<aos::logger::SnappyEncoder>(
        max_message_size, absl::GetFlag(FLAGS_flush_size));
  });

  aos::monotonic_clock::time_point last_rotation_time =
      event_loop.monotonic_now();
  aos::logger::Logger logger(
      &event_loop, event_loop.configuration(),
      // Only log channels smaller than ~10 MB / sec.
      [](const aos::Channel *channel) {
        return (channel->max_size() * channel->frequency()) < 10e6;
      });

  if (absl::GetFlag(FLAGS_rotate_every) != 0.0) {
    logger.set_on_logged_period(
        [&logger, &last_rotation_time](aos::monotonic_clock::time_point t) {
          if (t > last_rotation_time + std::chrono::duration<double>(
                                           absl::GetFlag(FLAGS_rotate_every))) {
            logger.Rotate();
            last_rotation_time = t;
          }
        });
  }

  event_loop.OnRun([&log_namer, &logger]() {
    errno = 0;
    setpriority(PRIO_PROCESS, 0, -20);
    PCHECK(errno == 0)
        << ": Renicing to -20 failed, use --skip_renicing to skip renicing.";
    logger.StartLogging(std::move(log_namer));
  });

  event_loop.Run();

  LOG(INFO) << "Shutting down";

  return 0;
}
