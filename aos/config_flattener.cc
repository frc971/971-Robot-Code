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
  CHECK_GE(argc, 3) << ": Too few arguments";

  VLOG(1) << "Reading " << argv[2];
  FlatbufferDetachedBuffer<Configuration> config =
      configuration::ReadConfig(argv[2]);

  std::vector<aos::FlatbufferString<reflection::Schema>> schemas;

  for (int i = 3; i < argc; ++i) {
    schemas.emplace_back(util::ReadFileToStringOrDie(argv[i]));
  }

  const std::string merged_config = FlatbufferToJson(
      &configuration::MergeConfiguration(config, schemas).message(), true);

  // TODO(austin): Figure out how to squash the schemas onto 1 line so it is
  // easier to read?  Or figure out how to split them into a second file which
  // gets included.
  VLOG(1) << "Flattened config is " << merged_config;
  util::WriteStringToFileOrDie(argv[1], merged_config);
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
