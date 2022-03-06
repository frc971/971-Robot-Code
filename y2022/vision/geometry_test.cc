#include "y2022/vision/geometry.h"

#include <cmath>

#include "aos/util/math.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace y2022::vision::testing {

TEST(GeometryTest, SlopeInterceptLine) {
  // Test a normal line
  {
    SlopeInterceptLine l({2.0, 3.0}, {4.0, 2.0});
    EXPECT_DOUBLE_EQ(l.m, -0.5);
    EXPECT_DOUBLE_EQ(l.b, 4.0);
    EXPECT_DOUBLE_EQ(l(5), 1.5);
  }
  // Test a horizontal line
  {
    SlopeInterceptLine l({2.0, 3.0}, {4.0, 3.0});
    EXPECT_DOUBLE_EQ(l.m, 0.0);
    EXPECT_DOUBLE_EQ(l.b, 3.0);
    EXPECT_DOUBLE_EQ(l(1000.0), 3.0);
  }
  // Test duplicate points
  {
    SlopeInterceptLine l({2.0, 3.0}, {2.0, 3.0});
    EXPECT_DOUBLE_EQ(l.m, 0.0);
    EXPECT_DOUBLE_EQ(l.b, 3.0);
    EXPECT_DOUBLE_EQ(l(1000.0), 3.0);
  }
  // Test infinite slope
  {
    EXPECT_DEATH(SlopeInterceptLine({2.0, 3.0}, {2.0, 5.0}),
                 "(.*)infinite slope(.*)");
  }
}

TEST(GeometryTest, StdFormLine) {
  // Test the intersection of normal lines
  {
    StdFormLine l{3.0, 2.0, 2.3};
    StdFormLine m{-2.0, 1.2, -0.3};
    const cv::Point2d kIntersection = {42.0 / 95.0, 37.0 / 76.0};
    EXPECT_EQ(*l.Intersection(m), kIntersection);
    EXPECT_EQ(*m.Intersection(l), kIntersection);
  }
  // Test the intersection of parallel lines
  {
    StdFormLine l{-3.0, 2.0, -3.7};
    StdFormLine m{-6.0, 4.0, 0.1};
    EXPECT_EQ(l.Intersection(m), std::nullopt);
    EXPECT_EQ(m.Intersection(l), std::nullopt);
  }
  // Test the intersection of duplicate lines
  {
    StdFormLine l{6.0, -8.0, 0.23};
    StdFormLine m{6.0, -8.0, 0.23};
    EXPECT_EQ(l.Intersection(m), std::nullopt);
    EXPECT_EQ(m.Intersection(l), std::nullopt);
  }
}

TEST(GeometryTest, Circle) {
  // Test fitting a normal circle
  {
    auto c = Circle::Fit({{-6.0, 3.2}, {-3.0, 2.0}, {-9.3, 1.4}});
    EXPECT_TRUE(c.has_value());
    EXPECT_NEAR(c->center.x, -5.901, 1e-3);
    EXPECT_NEAR(c->center.y, -0.905, 1e-3);
    EXPECT_NEAR(c->radius, 4.106, 1e-3);

    // Coordinate systems flipped because of image
    const cv::Point2d kPoint = {c->center.x - c->radius * std::sqrt(3.0) / 2.0,
                                c->center.y - c->radius / 2.0};
    EXPECT_NEAR(c->AngleOf(kPoint), 5.0 * M_PI / 6.0, 1e-5);
    EXPECT_TRUE(c->InAngleRange(kPoint, 4.0 * M_PI / 6.0, M_PI));
    EXPECT_FALSE(c->InAngleRange(kPoint, 0, 2.0 * M_PI / 6.0));
    EXPECT_EQ(c->DistanceTo(kPoint), 0.0);

    const cv::Point2d kZeroPoint = {c->center.x + c->radius, c->center.y};
    EXPECT_NEAR(c->AngleOf(kZeroPoint), 0.0, 1e-5);
    EXPECT_TRUE(
        c->InAngleRange(kZeroPoint, -2.1 * 2.0 * M_PI, -1.9 * 2.0 * M_PI));
    EXPECT_TRUE(
        c->InAngleRange(kZeroPoint, 1.9 * 2.0 * M_PI, 2.1 * 2.0 * M_PI));
    EXPECT_EQ(c->DistanceTo(kZeroPoint), 0.0);

    // Test the distance to another point
    const cv::Point2d kDoubleDistPoint = {
        c->center.x - (c->radius * 2.0) * std::sqrt(3.0) / 2.0,
        c->center.y - (c->radius * 2.0) / 2.0};
    EXPECT_DOUBLE_EQ(c->DistanceTo(kDoubleDistPoint), c->radius);

    // Distance to center should be radius
    EXPECT_DOUBLE_EQ(c->DistanceTo(c->center), c->radius);
  }
  // Test fitting an invalid circle (duplicate points)
  {
    auto c = Circle::Fit({{-6.0, 3.2}, {-3.0, 2.0}, {-6.0, 3.2}});
    EXPECT_FALSE(c.has_value());
  }
}

}  // namespace y2022::vision::testing
