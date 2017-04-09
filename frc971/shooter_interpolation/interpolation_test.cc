#include <unistd.h>

#include <memory>
#include <random>
#include <utility>

#include "gtest/gtest.h"

#include "frc971/shooter_interpolation/interpolation.h"

namespace frc971 {
namespace shooter_interpolation {

bool operator==(ShotParams a1, ShotParams a2) {
  return a1.angle == a2.angle && a1.power == a2.power;
}

// Tests to see if distances whose values are on the table are processed
// correctly
TEST(InterpolationTable, ExactNumbers) {
  ::std::vector<::std::pair<double, ShotParams>> data{
      {1, {10, 10}}, {3, {20, 20}}, {2, {15, 12345678}}, {4, {10, 567.323}},
  };

  InterpolationTable interpolation(data);
  ASSERT_EQ(data[1].second, interpolation.GetShooterData(3));
  ASSERT_EQ(data[3].second, interpolation.GetShooterData(4));
}

// Tests to see if distances whose values are off the table are processed
// correctly
TEST(InterpolationTable, InexactNumbers) {
  ::std::vector<::std::pair<double, ShotParams>> data{
      {1, {10, 10}}, {3, {20, 20}}, {2, {15, 15}}, {4, {10, 567.323}},
  };

  InterpolationTable interpolation(data);
  ASSERT_EQ(ShotParams({12.5, 12.5}), interpolation.GetShooterData(1.5));
  ASSERT_EQ(ShotParams({10, 10}), interpolation.GetShooterData(0));
}

// Tests to see if distances whose values are beyond the range of the table are
// processed correctly
TEST(InterpolationTable, OutOfScopeNumbers) {
  ::std::vector<::std::pair<double, ShotParams>> data{
      {1, {10, 10}}, {3, {20, 20}}, {2, {15, 12345678}}, {4, {10, 567.323}},
  };

  InterpolationTable interpolation(data);
  ASSERT_EQ(ShotParams({10, 10}), interpolation.GetShooterData(0));
  ASSERT_EQ(ShotParams({10, 567.323}), interpolation.GetShooterData(5));
}

}  // namespace shooter_interpolation
}  // namespace frc971
