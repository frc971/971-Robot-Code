#include "aos/init.h"
#include "aos/util/file.h"
#include "gflags/gflags.h"
#include "y2023/localizer/map_expander_lib.h"

DEFINE_string(target_map, "y2023/vision/maps/target_map.json",
              "Path to the target map JSON file.");
DEFINE_string(relative_map, "y2023/constants/relative_scoring_map.json",
              "Path to the relative scoring map JSON file.");
DEFINE_string(output, "y2023/constants/scoring_map.json",
              "Path to the output scoring map JSON file.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<y2023::localizer::ScoringMap> map =
      y2023::localizer::ExpandMap(
          aos::util::ReadFileToStringOrDie(FLAGS_relative_map),
          aos::util::ReadFileToStringOrDie(FLAGS_target_map));
  aos::util::WriteStringToFileOrDie(
      FLAGS_output,
      aos::FlatbufferToJson(map, {.multi_line = true, .max_multi_line = true}));
  return EXIT_SUCCESS;
}
