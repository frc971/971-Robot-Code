#include "aos/util/math.h"

#include "gtest/gtest.h"

namespace aos {
namespace math {
namespace testing {

bool AngleEqual(double a1, double a2) {
  double diff = a1 - a2;
  return ::std::fmod(diff, 2 * M_PI) == 0.0;
}

bool AngleInBounds(double a) { return a <= M_PI && a > -M_PI; }

// Check that something
void ExpectNormalizesCorrectly(double a) {
  double b = NormalizeAngle(a);
  EXPECT_PRED2(AngleEqual, a, b);
  EXPECT_PRED1(AngleInBounds, b);
}

void ExpectDiffsCorrectly(double a, double b) {
  double diff = DiffAngle(a, b);

  EXPECT_PRED1(AngleInBounds, diff) << "a: " << a << " b: " << b;
  EXPECT_PRED2(AngleEqual, a, b + diff);
}

// Checks that normalizing positive and negative angles in and out of bounds
// works correctly.
TEST(MathTest, NormalizeAngleTest) {
  ExpectNormalizesCorrectly(0.0);
  ExpectNormalizesCorrectly(1.0);
  ExpectNormalizesCorrectly(-1.0);
  ExpectNormalizesCorrectly(M_PI);
  ExpectNormalizesCorrectly(2.0 * M_PI);
  ExpectNormalizesCorrectly(-2.0 * M_PI);
  ExpectNormalizesCorrectly(100.0);
  ExpectNormalizesCorrectly(-100.0);
}

// Checks that subtracting one angle from another works for various combinations
// of angles.
TEST(MathTest, DiffAngleTest) {
  ExpectDiffsCorrectly(0.0, 0.0);
  ExpectDiffsCorrectly(1.0, 0.0);
  ExpectDiffsCorrectly(0.0, 1.0);
  ExpectDiffsCorrectly(-1.0, 1.0);
  ExpectDiffsCorrectly(100.0, 1.0);
  ExpectDiffsCorrectly(-100.0, 1.0);
  ExpectDiffsCorrectly(M_PI, M_PI);
}

// Check that points that are really counter-clockwise show up as
// counter-clockwise, and that invalid orderings don't show up as CCW.
TEST(MathTest, PointsAreCCWTest) {
  Eigen::Vector2d a, b, c;
  a << 0.0, 0.0;
  b << 1.0, 0.0;
  c << 0.0, 1.0;
  // Because there are only six orderings we can just enumerate them all.
  EXPECT_TRUE(PointsAreCCW<double>(a, b, c));
  EXPECT_FALSE(PointsAreCCW<double>(a, c, b));
  EXPECT_FALSE(PointsAreCCW<double>(b, a, c));
  EXPECT_TRUE(PointsAreCCW<double>(b, c, a));
  EXPECT_TRUE(PointsAreCCW<double>(c, a, b));
  EXPECT_FALSE(PointsAreCCW<double>(c, b, a));
}

// Collinear points should always appear as non-counter-clockwise.
TEST(MathTest, CollinearPointsAreCCWTest) {
  // Create three collinear points along the X-axis and check that every
  // combination is not CCW.
  Eigen::Vector2d a, b, c;
  a << 0.0, 0.0;
  b << 1.0, 0.0;
  c << 2.0, 0.0;
  EXPECT_FALSE(PointsAreCCW<double>(a, b, c));
  EXPECT_FALSE(PointsAreCCW<double>(a, c, b));
  EXPECT_FALSE(PointsAreCCW<double>(b, a, c));
  EXPECT_FALSE(PointsAreCCW<double>(b, c, a));
  EXPECT_FALSE(PointsAreCCW<double>(c, a, b));
  EXPECT_FALSE(PointsAreCCW<double>(c, b, a));
}

}  // namespace testing
}  // namespace math
}  // namespace aos
