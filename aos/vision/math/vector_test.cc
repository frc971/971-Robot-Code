#include "aos/vision/math/vector.h"

#include "gtest/gtest.h"

namespace aos {
namespace vision {
namespace testing {

class VectorTest : public ::testing::Test {
 protected:
  const Vector<3> vec1_{1.0, 1.0, 1.0};
  const Vector<3> vec2_{2.0, 4.0, 6.0};
  const Vector<3> vec5_{2.0, 2.0, 1.0};
  const Vector<3> vec6_{1.0, 1.0, 1.0};
};

TEST_F(VectorTest, Equality) {
  EXPECT_FALSE(vec1_ == vec2_);
  EXPECT_TRUE(Vector<3>(2.0, 4.0, 6.0) == vec2_);
}

TEST_F(VectorTest, Addition) {
  Vector<3> vec3 = vec1_ + vec2_;
  EXPECT_EQ(3.0, vec3.x());
  EXPECT_EQ(5.0, vec3.y());
  EXPECT_EQ(7.0, vec3.z());
}

TEST_F(VectorTest, Multiplication) {
  auto new_vec1 = vec1_;
  new_vec1 *= 2.0;
  EXPECT_EQ(2.0, new_vec1.x());
  EXPECT_EQ(2.0, new_vec1.y());
  EXPECT_EQ(2.0, new_vec1.z());

  auto new_vec2 = new_vec1 * 2;
  EXPECT_EQ(4.0, new_vec2.x());
  EXPECT_EQ(4.0, new_vec2.y());
  EXPECT_EQ(4.0, new_vec2.z());
}

TEST_F(VectorTest, Magnitude) {
  const Vector<3> vec4(1.0, 1.0, 1.0);
  EXPECT_NEAR(1.732, vec4.Mag(), 0.001);

  EXPECT_NEAR(1.414, (vec5_ - vec6_).Mag(), 0.001);
  EXPECT_NEAR(1.414, (vec6_ - vec5_).Mag(), 0.001);
}

TEST_F(VectorTest, Sign) {
  const Vector<3> vec7 = vec5_ - vec6_;
  const Vector<3> vec8 = vec6_ - vec5_;
  EXPECT_EQ(1.0, vec7.x());
  EXPECT_EQ(1.0, vec7.y());
  EXPECT_EQ(0.0, vec7.z());
  EXPECT_EQ(-1.0, vec8.x());
  EXPECT_EQ(-1.0, vec8.y());
  EXPECT_EQ(0.0, vec8.z());
}

TEST_F(VectorTest, Angle) {
  const Vector<3> vec1(1.0, 0.0, 0.0);
  const Vector<3> vec2(0.0, 1.0, 0.0);
  EXPECT_NEAR(M_PI / 2, vec1.AngleTo(vec2), 0.0001);
  const Vector<3> vec3 = Vector<3>(1.0, 1.0, 0.0);
  EXPECT_NEAR(M_PI / 4, vec1.AngleTo(vec3), 0.0001);

  const Vector<2> vec4(1, 1);
  EXPECT_NEAR(M_PI / 4, vec4.AngleToZero(), 0.0001);
  const Vector<2> vec5(0.5, 0.8660254037844386);
  EXPECT_NEAR(M_PI / 3, vec5.AngleToZero(), 0.0001);
}

}  // namespace testing
}  // namespace vision
}  // namespace aos
