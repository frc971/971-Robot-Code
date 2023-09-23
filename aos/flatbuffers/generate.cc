#include "flatbuffers/reflection_generated.h"

#include "aos/flatbuffers.h"
#include "aos/flatbuffers/static_flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"

DEFINE_string(reflection_bfbs, "", "Path to the .bfbs reflection file.");
DEFINE_string(output_file, "", "Path to the output header to write.");

namespace aos::fbs {
int Main() {
  aos::FlatbufferVector<reflection::Schema> schema =
      aos::FileToFlatbuffer<reflection::Schema>(FLAGS_reflection_bfbs);
  aos::util::WriteStringToFileOrDie(
      FLAGS_output_file, GenerateCodeForRootTableFile(&schema.message()));
  return EXIT_SUCCESS;
}
}  // namespace aos::fbs
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::fbs::Main();
}
