#include <random>

#include "frc971/zeroing/wrap.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace zeroing {
namespace testing {

// Tests some various positive and negative values for wrap.
TEST(WrapTest, TestWrap) {
  EXPECT_NEAR(1.0, Wrap(0.0, 1.0, 10.0), 1e-6);
  EXPECT_NEAR(-1.0, Wrap(0.0, -1.0, 10.0), 1e-6);

  EXPECT_NEAR(1.0, Wrap(5.0, 1.0, 10.0), 1e-6);
  EXPECT_NEAR(9.0, Wrap(5.0, -1.0, 10.0), 1e-6);

  EXPECT_NEAR(10.0, Wrap(5.0, 10.0, 10.0), 1e-6);
  EXPECT_NEAR(1.0, Wrap(5.0, -9.0, 10.0), 1e-6);
}

// Try a bunch of combinations and verify that the result has the right
// properties.  We can be really inefficient here since it is a test.
TEST(WrapTest, ExhaustiveWrap) {
  for (double i = -20; i < 20; ++i) {
    for (double j = -20; j < 20; ++j) {
      EXPECT_NEAR(i, Wrap(i, j, 10.0), 5.0);
      const double wrapped_val = Wrap(i, j, 10.0);
      bool found_interval = false;
      for (int k = -5; k < 5; ++k) {
        if (::std::abs(k * 10 + wrapped_val - j) < 1e-6) {
          found_interval = true;
          break;
        }
      }
      EXPECT_TRUE(found_interval) << ": Wrap(" << i << ", " << j
                                  << ") = " << wrapped_val;
    }
  }
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
