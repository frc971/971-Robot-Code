#include "absl/flags/flag.h"

#include "aos/init.h"
#include "aos/util/file.h"
#include "y2023/localizer/map_expander_lib.h"

ABSL_FLAG(std::string, target_map, "y2023/vision/maps/target_map.json",
          "Path to the target map JSON file.");
ABSL_FLAG(std::string, relative_map,
          "y2023/constants/relative_scoring_map.json",
          "Path to the relative scoring map JSON file.");
ABSL_FLAG(std::string, output, "y2023/constants/scoring_map.json",
          "Path to the output scoring map JSON file.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<y2023::localizer::ScoringMap> map =
      y2023::localizer::ExpandMap(
          aos::util::ReadFileToStringOrDie(absl::GetFlag(FLAGS_relative_map)),
          aos::util::ReadFileToStringOrDie(absl::GetFlag(FLAGS_target_map)));
  aos::util::WriteStringToFileOrDie(
      absl::GetFlag(FLAGS_output),
      aos::FlatbufferToJson(map, {.multi_line = true, .max_multi_line = true}));
  return EXIT_SUCCESS;
}
