#include "motors/math.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace frc971 {
namespace salsa {
namespace testing {

class SinCosIntTest : public ::testing::Test {
 public:
  void SetUp() override {
    MathInit();
  }

  template <class Rotation>
  void CheckSinCos() {
    static constexpr float kTolerance = 0.004;
    for (uint32_t theta = 0; theta < Rotation::den; ++theta) {
      EXPECT_THAT(
          FastSinInt<Rotation>(theta),
          ::testing::FloatNear(sin(ThetaToFloat<Rotation>(theta)), kTolerance));
      EXPECT_THAT(
          FastCosInt<Rotation>(theta),
          ::testing::FloatNear(cos(ThetaToFloat<Rotation>(theta)), kTolerance));
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

TEST_F(SinCosIntTest, Numerator1) {
  CheckSinCos<::std::ratio<1, 2>>();
  CheckSinCos<::std::ratio<1, 4>>();
  CheckSinCos<::std::ratio<1, 8>>();
  CheckSinCos<::std::ratio<1, 128>>();
  CheckSinCos<::std::ratio<1, 1024>>();
  CheckSinCos<::std::ratio<1, 4096>>();
}

TEST_F(SinCosIntTest, Numerator5) {
  CheckSinCos<::std::ratio<5, 8>>();
  CheckSinCos<::std::ratio<5, 128>>();
  CheckSinCos<::std::ratio<5, 1024>>();
  CheckSinCos<::std::ratio<5, 4096>>();
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
}  // namespace salsa
}  // namespace frc971
