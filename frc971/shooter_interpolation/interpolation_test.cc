#include <unistd.h>

#include <memory>
#include <random>
#include <utility>

#include "gtest/gtest.h"

#include "frc971/shooter_interpolation/interpolation.h"

namespace frc971 {
namespace shooter_interpolation {

struct TestShotParams {
  double angle;
  double power;
  static TestShotParams BlendY(double x, const TestShotParams& a, const TestShotParams& b) {
    return TestShotParams{
      Blend(x, a.angle, b.angle),
      Blend(x, a.power, b.power)
    };
  }
};

bool operator==(TestShotParams a1, TestShotParams a2) {
  return a1.angle == a2.angle && a1.power == a2.power;
}

using TestInterpolationTable = InterpolationTable<TestShotParams>;

// Tests to see if distances whose values are on the table are processed
// correctly
TEST(InterpolationTable, ExactNumbers) {
  ::std::vector<::std::pair<double, TestShotParams>> data = {
      {1, {10, 10}}, {3, {20, 20}}, {2, {15, 12345678}}, {4, {10, 567.323}},
  };

  TestInterpolationTable interpolation(data);
  ASSERT_EQ(data[1].second, interpolation.Get(3));
  ASSERT_EQ(data[3].second, interpolation.Get(4));
}

// Tests to see if distances whose values are off the table are processed
// correctly
TEST(InterpolationTable, InexactNumbers) {
  ::std::vector<::std::pair<double, TestShotParams>> data = {
      {1, {10, 10}}, {3, {20, 20}}, {2, {15, 15}}, {4, {10, 567.323}},
  };

  TestInterpolationTable interpolation(data);
  ASSERT_EQ(TestShotParams({12.5, 12.5}), interpolation.Get(1.5));
  ASSERT_EQ(TestShotParams({10, 10}), interpolation.Get(0));
}

// Tests to see if distances whose values are beyond the range of the table are
// processed correctly
TEST(InterpolationTable, OutOfScopeNumbers) {
  ::std::vector<::std::pair<double, TestShotParams>> data = {
      {1, {10, 10}}, {3, {20, 20}}, {2, {15, 12345678}}, {4, {10, 567.323}},
  };

  TestInterpolationTable interpolation(data);
  ASSERT_EQ(TestShotParams({10, 10}), interpolation.Get(0));
  ASSERT_EQ(TestShotParams({10, 567.323}), interpolation.Get(5));
}

}  // namespace shooter_interpolation
}  // namespace frc971
