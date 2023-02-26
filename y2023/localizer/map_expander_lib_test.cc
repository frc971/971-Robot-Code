#include "y2023/localizer/map_expander_lib.h"

#include "gtest/gtest.h"
#include "aos/testing/flatbuffer_eq.h"

namespace y2023::localizer::testing {
class MapExpanderTest : public ::testing::Test {
 protected:
  MapExpanderTest()
      : relative_map_(aos::JsonFileToFlatbuffer<RelativeScoringMap>(
            "y2023/constants/relative_scoring_map.json")),
        target_map_(aos::JsonFileToFlatbuffer<frc971::vision::TargetMap>(
            "y2023/vision/maps/target_map.json")),
        absolute_map_(
            ExpandMap(&relative_map_.message(), &target_map_.message())) {}
  const aos::FlatbufferDetachedBuffer<RelativeScoringMap> relative_map_;
  const aos::FlatbufferDetachedBuffer<frc971::vision::TargetMap> target_map_;
  const aos::FlatbufferDetachedBuffer<ScoringMap> absolute_map_;
};
TEST_F(MapExpanderTest, BackAndForthConsistent) {
  // Use FlatbufferToJson instead of FlatbufferEq because we don't want
  // equivalent but different encoded floating point numbers to get
  // evaluated differently.
#define CHECK_REVERSE(color, grid)                                           \
  {                                                                          \
    ASSERT_TRUE(absolute_map_.message().has_##color());                      \
    auto half = absolute_map_.message().color();                             \
    ASSERT_TRUE(half->has_##grid##_grid());                                  \
    auto grid = half->grid##_grid();                                         \
    auto relative_grid =                                                     \
        RelativeGridForTag(grid, &target_map_.message(),                     \
                           relative_map_.message().color()->grid());         \
    EXPECT_EQ(aos::FlatbufferToJson(relative_map_.message().nominal_grid()), \
              aos::FlatbufferToJson(&relative_grid.message()));              \
  }
  CHECK_REVERSE(blue, left);
  CHECK_REVERSE(blue, middle);
  CHECK_REVERSE(blue, right);
  CHECK_REVERSE(red, left);
  CHECK_REVERSE(red, middle);
  CHECK_REVERSE(red, right);
}

// Test that the currently checked-in map is consistent with the results of
// ExpandMap.
TEST_F(MapExpanderTest, ExpandMap) {
  const std::string stored_map =
      aos::util::ReadFileToStringOrDie("y2023/constants/scoring_map.json");
  // TODO: Provide coherent error messages so that changes can be accommodated.
  EXPECT_EQ(aos::FlatbufferToJson(absolute_map_,
                                  {.multi_line = true, .max_multi_line = true}),
            stored_map);
}
}  // namespace y2023::localizer::testing
