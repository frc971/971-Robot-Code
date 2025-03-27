#include <sys/resource.h>
#include <sys/time.h>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/log_namer.h"
#include "aos/util/filesystem_generated.h"
#include "frc971/input/joystick_state_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Config file to use.");
ABSL_FLAG(double, rotate_every, 0.0,
          "If set, rotate the logger after this many seconds");
ABSL_DECLARE_FLAG(int32_t, flush_size);
ABSL_FLAG(double, disabled_time, 5.0,
          "Continue logging if disabled for this amount of time or less");
ABSL_FLAG(bool, direct, false,
          "If true, write using O_DIRECT and write 512 byte aligned blocks "
          "whenever possible.");
ABSL_FLAG(bool, use_one_orin, true, "Use one orin instead of two.");
std::unique_ptr<aos::logger::MultiNodeFilesLogNamer> MakeLogNamer(
    aos::EventLoop *event_loop) {
  std::optional<std::string> log_name =
      aos::logging::MaybeGetLogName("image_log");

  if (!log_name.has_value()) {
    return nullptr;
  }

  return std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
      event_loop,
      std::make_unique<aos::logger::RenamableFileBackend>(
          absl::StrCat(log_name.value(), "/"), absl::GetFlag(FLAGS_direct)));
}

int main(int argc, char *argv[]) {
  absl::SetProgramUsageMessage(
      "This program provides a simple logger binary that logs all SHMEM data "
      "directly to a file specified at the command line when the robot is "
      "enabled and for a bit of time after.");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  aos::Fetcher<aos::util::FilesystemStatus> filesystem_status =
      event_loop.MakeFetcher<aos::util::FilesystemStatus>("/aos");

  bool logging = false;
  bool enabled = false;
  aos::monotonic_clock::time_point last_disable_time =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point last_rotation_time =
      event_loop.monotonic_now();
  aos::logger::Logger logger(&event_loop);

  if (absl::GetFlag(FLAGS_rotate_every) != 0.0) {
    logger.set_on_logged_period([&](aos::monotonic_clock::time_point) {
      const auto now = event_loop.monotonic_now();
      if (logging &&
          now > last_rotation_time + std::chrono::duration<double>(
                                         absl::GetFlag(FLAGS_rotate_every))) {
        logger.Rotate();
        last_rotation_time = now;
      }
    });
  }

  LOG(INFO) << "Starting image_logger; will wait on joystick enabled to start "
               "logging";
  event_loop.OnRun([]() {
    errno = 0;
    setpriority(PRIO_PROCESS, 0, -20);
    PCHECK(errno == 0) << ": Renicing to -20 failed.";
  });

  event_loop.MakeWatcher(
      absl::GetFlag(FLAGS_use_one_orin) ? "/orin1/aos" : "/imu/aos",
      [&](const aos::JoystickState &joystick_state) {
        const auto timestamp = event_loop.context().monotonic_event_time;
        filesystem_status.Fetch();

        // Store the last time we got disabled
        if (enabled && !joystick_state.enabled()) {
          last_disable_time = timestamp;
        }
        enabled = joystick_state.enabled();

        bool enough_space = true;

        if (filesystem_status.get() != nullptr) {
          enough_space = false;
          for (const aos::util::Filesystem *fs :
               *filesystem_status->filesystems()) {
            CHECK(fs->has_path());
            if (fs->path()->string_view() == "/") {
              if (fs->free_space() > 50ull * 1024ull * 1024ull * 1024ull) {
                enough_space = true;
              }
            }
          }
        }

        const bool should_be_logging =
            (enabled ||
             timestamp <
                 last_disable_time + std::chrono::duration<double>(
                                         absl::GetFlag(FLAGS_disabled_time))) &&
            enough_space;

        if (!logging && should_be_logging) {
          auto log_namer = MakeLogNamer(&event_loop);
          if (log_namer == nullptr) {
            return;
          }

          // Start logging if we just got enabled
          LOG(INFO) << "Starting logging";
          logger.StartLogging(std::move(log_namer));
          logging = true;
          last_rotation_time = event_loop.monotonic_now();
        } else if (logging && !should_be_logging) {
          // Stop logging if we've been disabled for a non-negligible amount of
          // time
          LOG(INFO) << "Stopping logging";
          logger.StopLogging(event_loop.monotonic_now());
          logging = false;
        }
      });

  event_loop.Run();

  LOG(INFO) << "Shutting down";

  return 0;
}
