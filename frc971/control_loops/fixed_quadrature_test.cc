#include "frc971/control_loops/fixed_quadrature.h"

#include <cmath>

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// Tests that integrating y = cos(x) works.
TEST(GaussianQuadratureTest, Cos) {
  double y1 =
      GaussianQuadrature5([](double x) { return ::std::cos(x); }, 0.0, 0.5);

  EXPECT_NEAR(y1, ::std::sin(0.5), 1e-15);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
