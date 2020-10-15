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
  CHECK_GE(argc, 6) << ": Too few arguments";

  const char *full_output = argv[1];
  const char *stripped_output = argv[2];
  const char *binary_output = argv[3];
  const char *config_path = argv[4];
  // In order to support not only importing things by absolute path, but also
  // importing the outputs of genrules (rather than just manually written
  // files), we need to tell ReadConfig where the generated files directory is.
  const char *bazel_outs_directory = argv[5];

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

  for (int i = 6; i < argc; ++i) {
    schemas.emplace_back(util::ReadFileToStringOrDie(argv[i]));
  }

  aos::FlatbufferDetachedBuffer<Configuration> merged_config =
      configuration::MergeConfiguration(config, schemas);

  const std::string merged_config_json =
      FlatbufferToJson(&merged_config.message(), {.multi_line = true});

  // TODO(austin): Figure out how to squash the schemas onto 1 line so it is
  // easier to read?
  VLOG(1) << "Flattened config is " << merged_config_json;
  util::WriteStringToFileOrDie(full_output, merged_config_json);
  util::WriteStringToFileOrDie(
      stripped_output,
      FlatbufferToJson(&config.message(), {.multi_line = true}));
  aos::WriteFlatbufferToFile(binary_output, merged_config);
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
