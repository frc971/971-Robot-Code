#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/constants/constants_list_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Path to the AOS config.");
ABSL_FLAG(std::string, constants_path, "constants.json",
          "Path to the constant file");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());
  frc971::constants::ConstantSender<y2024_bot3::Constants,
                                    y2024_bot3::ConstantsList>
      constants_sender(&event_loop, absl::GetFlag(FLAGS_constants_path));
  // Don't need to call Run().
  return 0;
}
