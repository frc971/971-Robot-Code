#include "motors/math.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace frc971 {
namespace motors {
namespace testing {

class SinCosIntTest : public ::testing::Test {
 public:
  void SetUp() override {
    MathInit();
  }

  template <class Rotation, int kTableSize>
  void CheckSinCos() {
    static constexpr float kTolerance = 0.004;
    SCOPED_TRACE("num=" + ::std::to_string(Rotation::num) + " den=" +
                 ::std::to_string(Rotation::den) + " table_size=" +
                 ::std::to_string(kTableSize));
    for (uint32_t theta = 0; theta < Rotation::den; ++theta) {
      const float theta_float = ThetaToFloat<Rotation>(theta);
      SCOPED_TRACE("theta=" + ::std::to_string(theta) + " theta_float=" +
                   ::std::to_string(theta_float));
      EXPECT_THAT((FastSinInt<Rotation, kTableSize>(theta)),
                  ::testing::FloatNear(sin(theta_float), kTolerance));
      EXPECT_THAT((FastCosInt<Rotation, kTableSize>(theta)),
                  ::testing::FloatNear(cos(theta_float), kTolerance));
    }
  }

 private:
  template <class Rotation>
  double ThetaToFloat(uint32_t theta) {
    const double rotation_double =
        static_cast<double>(Rotation::num) / static_cast<double>(Rotation::den);
    return (static_cast<double>(theta) + 0.5) * rotation_double * 2.0 * M_PI;
  }
};

TEST_F(SinCosIntTest, Numerator1DefaultTable) {
  CheckSinCos<::std::ratio<1, 2>, 2>();
  CheckSinCos<::std::ratio<1, 4>, 4>();
  CheckSinCos<::std::ratio<1, 8>, 8>();
  CheckSinCos<::std::ratio<1, 128>, 128>();
  CheckSinCos<::std::ratio<1, 1024>, 1024>();
  CheckSinCos<::std::ratio<1, 4096>, 4096>();
}

TEST_F(SinCosIntTest, Numerator1LargerTable) {
  // Don't include silly things like a denominator of 2 and table size of 4
  // here. It will fail because the slope varies so much that linearly
  // interpolating between 0.5pi/1.5pi vs 0.25pi/0.75pi/1.25pi/1.75pi gives very
  // different results.
  CheckSinCos<::std::ratio<1, 2>, 4096>();
  CheckSinCos<::std::ratio<1, 2>, 1024>();
  CheckSinCos<::std::ratio<1, 8>, 4096>();
  CheckSinCos<::std::ratio<1, 128>, 4096>();
  CheckSinCos<::std::ratio<1, 1024>, 2048>();
}

TEST_F(SinCosIntTest, Numerator5DefaultTable) {
  CheckSinCos<::std::ratio<5, 8>, 8>();
  CheckSinCos<::std::ratio<5, 128>, 128>();
  CheckSinCos<::std::ratio<5, 1024>, 1024>();
  CheckSinCos<::std::ratio<5, 4096>, 4096>();
}

TEST_F(SinCosIntTest, Numerator5LargerTable) {
  CheckSinCos<::std::ratio<5, 2>, 4096>();
  CheckSinCos<::std::ratio<5, 2>, 1024>();
  CheckSinCos<::std::ratio<5, 8>, 4096>();
  CheckSinCos<::std::ratio<5, 128>, 4096>();
  CheckSinCos<::std::ratio<5, 1024>, 2048>();
}

class SinCosFloatTest : public ::testing::Test {
 public:
  void SetUp() override {
    MathInit();
  }

  void CheckSinCos(float theta) {
    ASSERT_GE(theta, -0.2f);
    ASSERT_LE(theta, 0.2f);

    static constexpr float kTolerance = 0.002;
    EXPECT_THAT(FastSinFloat(theta),
                ::testing::FloatNear(sin(theta), kTolerance));
    EXPECT_THAT(FastCosFloat(theta),
                ::testing::FloatNear(cos(theta), kTolerance));
  }
};

TEST_F(SinCosFloatTest, Endpoints) {
  CheckSinCos(0);
  CheckSinCos(-0.2);
  CheckSinCos(0.2);
}

}  // namespace testing
}  // namespace motors
}  // namespace frc971
