#include "gtest/gtest.h"

#include "aos/atom_code/output/motor_output.h"

namespace aos {
namespace testing {

TEST(MotorControllerBoundsTest, Limits) {
  MotorOutput::MotorControllerBounds test_bounds
      {200, 135, 128, 126, 110};
  EXPECT_EQ(test_bounds.Map(1.0), test_bounds.kMax);
  EXPECT_EQ(test_bounds.Map(-1.0), test_bounds.kMin);
  EXPECT_EQ(test_bounds.Map(0.0), test_bounds.kCenter);
  EXPECT_EQ(test_bounds.Map(0.55), 171);
  EXPECT_EQ(test_bounds.Map(0.5), 168);
  EXPECT_EQ(test_bounds.Map(0.45), 164);
  EXPECT_EQ(test_bounds.Map(-0.5), 118);
}

}  // namespace testing
}  // namespace aos
