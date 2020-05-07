#include <string>
#include <vector>

#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace aos {

int Main(int argc, char **argv) {
  CHECK_GE(argc, 4) << ": Too few arguments";

  const char *full_output = argv[1];
  const char *stripped_output = argv[2];
  const char *config_path = argv[3];

  VLOG(1) << "Reading " << config_path;
  FlatbufferDetachedBuffer<Configuration> config =
      configuration::ReadConfig(config_path);

  std::vector<aos::FlatbufferString<reflection::Schema>> schemas;

  for (int i = 4; i < argc; ++i) {
    schemas.emplace_back(util::ReadFileToStringOrDie(argv[i]));
  }

  const std::string merged_config = FlatbufferToJson(
      &configuration::MergeConfiguration(config, schemas).message(),
      {.multi_line = true});

  // TODO(austin): Figure out how to squash the schemas onto 1 line so it is
  // easier to read?
  VLOG(1) << "Flattened config is " << merged_config;
  util::WriteStringToFileOrDie(full_output, merged_config);
  util::WriteStringToFileOrDie(
      stripped_output,
      FlatbufferToJson(&config.message(), {.multi_line = true}));
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
