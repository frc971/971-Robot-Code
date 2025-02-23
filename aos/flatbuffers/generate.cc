#include <stdlib.h>

#include "absl/flags/flag.h"
#include "flatbuffers/reflection_generated.h"

#include "aos/flatbuffers.h"
#include "aos/flatbuffers/static_flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"

ABSL_FLAG(std::string, reflection_bfbs, "",
          "Path to the .bfbs reflection file.");
ABSL_FLAG(std::string, output_file, "", "Path to the output header to write.");
ABSL_FLAG(std::string, base_file_name, "",
          "Name of the base file to generate code for as used by the "
          "reflection::Schema object.");

namespace aos::fbs {
int Main() {
  aos::FlatbufferVector<reflection::Schema> schema =
      aos::FileToFlatbuffer<reflection::Schema>(
          absl::GetFlag(FLAGS_reflection_bfbs));
  aos::util::WriteStringToFileOrDie(
      absl::GetFlag(FLAGS_output_file),
      GenerateCodeForRootTableFile(&schema.message(),
                                   absl::GetFlag(FLAGS_base_file_name)));
  return EXIT_SUCCESS;
}
}  // namespace aos::fbs
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::fbs::Main();
}
