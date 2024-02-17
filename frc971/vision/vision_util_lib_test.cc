#include "frc971/vision/vision_util_lib.h"

#include "gtest/gtest.h"

namespace frc971::vision {
// For now, just testing extracting camera number from channel name
TEST(VisionUtilsTest, CameraNumberFromChannel) {
  ASSERT_EQ(CameraNumberFromChannel("/camera0").value(), 0);
  ASSERT_EQ(CameraNumberFromChannel("/camera1").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/camera"), std::nullopt);
  ASSERT_EQ(CameraNumberFromChannel("/orin1/camera0").value(), 0);
  ASSERT_EQ(CameraNumberFromChannel("/orin1/camera1").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/orin1"), std::nullopt);
}
}  // namespace frc971::vision
