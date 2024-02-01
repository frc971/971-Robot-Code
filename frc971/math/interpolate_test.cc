#include "frc971/math/interpolate.h"

#include "gtest/gtest.h"
#include <Eigen/Core>

namespace frc971::math::testing {
// Tests that when we "interpolate" exactly to the ends of the interpolation
// that we get the correct result.
TEST(InterpolateTest, ExactEnds) {
  const Eigen::Vector3d a(1.0, 2.0, 3.0), b(4.0, 5.0, 6.0);
  ASSERT_EQ(a, lerp(a, b, 0.0));
  ASSERT_EQ(Eigen::Vector3d(2.5, 3.5, 4.5), lerp(a, b, 0.5));
  ASSERT_EQ(b, lerp(a, b, 1.0));

  ASSERT_EQ(a, Interpolate(10.0, 20.0, a, b, 10.0));
  ASSERT_EQ(b, Interpolate(10.0, 20.0, a, b, 20.0));
  ASSERT_EQ(Eigen::Vector3d(2.5, 3.5, 4.5),
            Interpolate(10.0, 20.0, a, b, 15.0));
}
}  // namespace frc971::math::testing
