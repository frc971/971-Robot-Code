#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "constants_sender_lib.h"
#include "frc971/constants/testdata/constants_data_generated.h"
#include "frc971/constants/testdata/constants_list_generated.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "frc971/constants/testdata/aos_config.json",
              "Path to the config.");
DEFINE_string(constants_path, "frc971/constants/testdata/test_constants.json",
              "Path to the constant file");
// This is just a sample binary
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);
  aos::ShmEventLoop event_loop(&config.message());
  frc971::constants::ConstantSender<frc971::constants::testdata::ConstantsData,
                                    frc971::constants::testdata::ConstantsList>
      constants_sender(&event_loop, FLAGS_constants_path);
  event_loop.Run();

  return 0;
}
