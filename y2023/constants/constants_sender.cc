#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/constants/constants_sender_lib.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/constants/constants_list_generated.h"

DEFINE_string(config, "aos_config.json", "Path to the AOS config.");
DEFINE_string(constants_path, "constants.json", "Path to the constant file");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);
  aos::ShmEventLoop event_loop(&config.message());
  frc971::constants::ConstantSender<y2023::Constants, y2023::ConstantsList>
      constants_sender(&event_loop, FLAGS_constants_path);
  // Don't need to call Run().
  return 0;
}
