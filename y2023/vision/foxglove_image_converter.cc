#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/vision/foxglove_image_converter_lib.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  frc971::vision::FoxgloveImageConverter converter(
      &event_loop, "/camera", "/camera",
      frc971::vision::ImageCompression::kJpeg);

  event_loop.Run();
}
