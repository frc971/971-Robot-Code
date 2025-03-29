#include <filesystem>
#include <iostream>

#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frame_logger.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

namespace y2025 {
namespace vision {
namespace {

void FrameLoggerMain() {
  std::string dir_name = "/home/pi/images";

  if (std::filesystem::exists(dir_name) ||
      std::filesystem::create_directory(dir_name)) {
    VLOG(0) << "Directory for frame data found.";
  } else {
    VLOG(1) << "Failed to create directory for frame data.";
    return;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  FrameLogger frame_logger(&event_loop);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2025

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2025::vision::FrameLoggerMain();
}
