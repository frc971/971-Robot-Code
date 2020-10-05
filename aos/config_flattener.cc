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
  CHECK_GE(argc, 5) << ": Too few arguments";

  const char *full_output = argv[1];
  const char *stripped_output = argv[2];
  const char *config_path = argv[3];
  // In order to support not only importing things by absolute path, but also
  // importing the outputs of genrules (rather than just manually written
  // files), we need to tell ReadConfig where the generated files directory is.
  const char *bazel_outs_directory = argv[4];

  VLOG(1) << "Reading " << config_path;
  FlatbufferDetachedBuffer<Configuration> config =
      configuration::ReadConfig(config_path, {bazel_outs_directory});

  for (const Channel *channel : *config.message().channels()) {
    if (channel->max_size() % alignof(flatbuffers::largest_scalar_t) != 0) {
      LOG(FATAL) << "max_size() (" << channel->max_size()
                 << ") is not a multiple of alignment ("
                 << alignof(flatbuffers::largest_scalar_t) << ") for channel "
                 << configuration::CleanedChannelToString(channel) << ".";
    }
  }

  std::vector<aos::FlatbufferString<reflection::Schema>> schemas;

  for (int i = 5; i < argc; ++i) {
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
