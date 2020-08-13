#include "aos/init.h"
#include "gflags/gflags.h"
#include "starterd_lib.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  const aos::Configuration *config_msg = &config.message();

  aos::starter::Starter starter(config_msg);

  starter.Run();

  return 0;
}
