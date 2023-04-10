#include <sys/resource.h>
#include <sys/time.h>

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/log_namer.h"
#include "frc971/input/joystick_state_generated.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "aos_config.json", "Config file to use.");

DEFINE_double(rotate_every, 0.0,
              "If set, rotate the logger after this many seconds");
DECLARE_int32(flush_size);
DEFINE_double(disabled_time, 5.0,
              "Continue logging if disabled for this amount of time or less");

std::unique_ptr<aos::logger::MultiNodeFilesLogNamer> MakeLogNamer(
    aos::EventLoop *event_loop) {
  std::optional<std::string> log_name =
      aos::logging::MaybeGetLogName("fbs_log");

  if (!log_name.has_value()) {
    return nullptr;
  }

  return std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
      absl::StrCat(log_name.value(), "/"), event_loop);
}

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage(
      "This program provides a simple logger binary that logs all SHMEM data "
      "directly to a file specified at the command line when the robot is "
      "enabled and for a bit of time after.");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  bool logging = false;
  bool enabled = false;
  aos::monotonic_clock::time_point last_disable_time =
      event_loop.monotonic_now();
  aos::monotonic_clock::time_point last_rotation_time =
      event_loop.monotonic_now();
  aos::logger::Logger logger(&event_loop);

  if (FLAGS_rotate_every != 0.0) {
    logger.set_on_logged_period([&] {
      const auto now = event_loop.monotonic_now();
      if (logging && now > last_rotation_time + std::chrono::duration<double>(
                                                    FLAGS_rotate_every)) {
        logger.Rotate();
        last_rotation_time = now;
      }
    });
  }

  event_loop.OnRun([]() {
    errno = 0;
    setpriority(PRIO_PROCESS, 0, -20);
    PCHECK(errno == 0) << ": Renicing to -20 failed.";
  });

  event_loop.MakeWatcher(
      "/imu/aos", [&](const aos::JoystickState &joystick_state) {
        const auto timestamp = event_loop.context().monotonic_event_time;
        // Store the last time we got disabled
        if (enabled && !joystick_state.enabled()) {
          last_disable_time = timestamp;
        }
        enabled = joystick_state.enabled();

        if (!logging && enabled) {
          auto log_namer = MakeLogNamer(&event_loop);
          if (log_namer == nullptr) {
            return;
          }

          // Start logging if we just got enabled
          LOG(INFO) << "Starting logging";
          logger.StartLogging(std::move(log_namer));
          logging = true;
          last_rotation_time = event_loop.monotonic_now();
        } else if (logging && !enabled &&
                   (timestamp - last_disable_time) >
                       std::chrono::duration<double>(FLAGS_disabled_time)) {
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
