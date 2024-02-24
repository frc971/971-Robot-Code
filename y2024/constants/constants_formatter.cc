#include "glog/logging.h"

#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"
#include "y2024/constants/constants_list_generated.h"

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  CHECK(argc == 3) << ": Expected input and output json files to be passed in.";

  aos::FlatbufferDetachedBuffer<y2024::ConstantsList> constants =
      aos::JsonFileToFlatbuffer<y2024::ConstantsList>(argv[1]);

  // Make sure the file is valid json before we output a formatted version.
  CHECK(constants.message().constants() != nullptr)
      << ": Failed to parse " << std::string(argv[2]);

  aos::util::WriteStringToFileOrDie(
      std::string(argv[2]),
      aos::FlatbufferToJson(constants, {.multi_line = true}));

  return 0;
}
