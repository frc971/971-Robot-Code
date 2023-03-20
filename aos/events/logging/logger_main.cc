#include <sys/resource.h>
#include <sys/time.h>

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#ifdef LZMA
#include "aos/events/logging/lzma_encoder.h"
#endif
#include "aos/events/logging/snappy_encoder.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/log_namer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "aos_config.json", "Config file to use.");

DEFINE_bool(skip_renicing, false,
            "If true, skip renicing the logger.  This leaves it lower priority "
            "and increases the likelihood of dropping messages and crashing.");

DEFINE_bool(snappy_compress, false, "If true, compress log data using snappy.");

#ifdef LZMA
DEFINE_bool(xz_compress, false, "If true, compress log data using xz.");
#endif

DEFINE_double(rotate_every, 0.0,
              "If set, rotate the logger after this many seconds");

#ifdef LZMA
DEFINE_int32(xz_compression_level, 9, "Compression level for the LZMA Encoder");
#endif

DECLARE_int32(flush_size);

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

  auto log_namer = std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
      absl::StrCat(aos::logging::GetLogName("fbs_log"), "/"), &event_loop);

  if (FLAGS_snappy_compress) {
    log_namer->set_extension(aos::logger::SnappyDecoder::kExtension);
    log_namer->set_encoder_factory([](size_t max_message_size) {
      return std::make_unique<aos::logger::SnappyEncoder>(max_message_size,
                                                          FLAGS_flush_size);
    });
#ifdef LZMA
  } else if (FLAGS_xz_compress) {
    log_namer->set_extension(aos::logger::LzmaEncoder::kExtension);
    log_namer->set_encoder_factory([](size_t max_message_size) {
      return std::make_unique<aos::logger::LzmaEncoder>(
          max_message_size, FLAGS_xz_compression_level, FLAGS_flush_size);
    });
#endif
  }

  aos::monotonic_clock::time_point last_rotation_time =
      event_loop.monotonic_now();
  aos::logger::Logger logger(&event_loop);

  if (FLAGS_rotate_every != 0.0) {
    logger.set_on_logged_period([&] {
      const auto now = event_loop.monotonic_now();
      if (now > last_rotation_time +
                    std::chrono::duration<double>(FLAGS_rotate_every)) {
        logger.Rotate();
        last_rotation_time = now;
      }
    });
  }

  event_loop.OnRun([&log_namer, &logger]() {
    if (FLAGS_skip_renicing) {
      LOG(WARNING) << "Ignoring request to renice to -20 due to "
                      "--skip_renicing.";
    } else {
      errno = 0;
      setpriority(PRIO_PROCESS, 0, -20);
      PCHECK(errno == 0)
          << ": Renicing to -20 failed, use --skip_renicing to skip renicing.";
    }
    logger.StartLogging(std::move(log_namer));
  });

  event_loop.Run();

  LOG(INFO) << "Shutting down";

  return 0;
}
