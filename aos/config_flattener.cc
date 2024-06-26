#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"

ABSL_FLAG(std::string, full_output, "",
          "If provided, the path to write the full json configuration to.");
ABSL_FLAG(std::string, stripped_output, "",
          "If provided, the path to write the stripped json configuration to.");
ABSL_FLAG(
    std::string, binary_output, "",
    "If provided, the path to write the binary flatbuffer configuration to.");

namespace aos {

int Main(int argc, char **argv) {
  CHECK_GE(argc, 3) << ": Too few arguments";

  const char *config_path = argv[1];
  // In order to support not only importing things by absolute path, but also
  // importing the outputs of genrules (rather than just manually written
  // files), we need to tell ReadConfig where the generated files directory is.
  const char *bazel_outs_directory = argv[2];

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

  std::vector<aos::FlatbufferVector<reflection::Schema>> schemas;

  for (int i = 3; i < argc; ++i) {
    auto schema = FileToFlatbuffer<reflection::Schema>(argv[i]);
    if (!schema.message().has_root_table()) {
      LOG(ERROR) << "Schema in " << argv[i]
                 << " does not have a root table, aborting";
      return 1;
    }
    schemas.emplace_back(std::move(schema));
  }

  aos::FlatbufferDetachedBuffer<Configuration> merged_config =
      configuration::MergeConfiguration(config, schemas);
  // In case we've done something overly weird to the flatbuffer...
  CHECK(merged_config.Verify())
      << ": Failed to verify flatbuffer. NOTE: Very large flatbuffers could be "
         "exceeding max_tables in flatbuffers::Verifier.";

  const std::string merged_config_json =
      FlatbufferToJson(&merged_config.message(), {.multi_line = true});

  // TODO(austin): Figure out how to squash the schemas onto 1 line so it is
  // easier to read?
  VLOG(1) << "Flattened config is " << merged_config_json;
  if (!absl::GetFlag(FLAGS_full_output).empty()) {
    util::WriteStringToFileOrDie(absl::GetFlag(FLAGS_full_output),
                                 merged_config_json);
  }
  if (!absl::GetFlag(FLAGS_stripped_output).empty()) {
    util::WriteStringToFileOrDie(
        absl::GetFlag(FLAGS_stripped_output),
        FlatbufferToJson(&config.message(), {.multi_line = true}));
  }
  if (!absl::GetFlag(FLAGS_binary_output).empty()) {
    aos::WriteFlatbufferToFile(absl::GetFlag(FLAGS_binary_output),
                               merged_config);
  }
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
