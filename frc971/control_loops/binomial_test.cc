#include "frc971/control_loops/binomial.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// Tests some factorials.
TEST(BinomialTest, Factorial) {
  EXPECT_EQ(1.0, Factorial(0));
  EXPECT_EQ(1.0, Factorial(1));
  EXPECT_EQ(2.0, Factorial(2));
  EXPECT_EQ(6.0, Factorial(3));
  EXPECT_EQ(24.0, Factorial(4));
}

// Test a 2nd order polynomial.
TEST(BinomialTest, Quadratic) {
  EXPECT_EQ(1.0, Binomial(2, 0));
  EXPECT_EQ(2.0, Binomial(2, 1));
  EXPECT_EQ(1.0, Binomial(2, 2));
}

// Test a 3th order polynomial.
TEST(BinomialTest, Cubic) {
  EXPECT_EQ(1.0, Binomial(3, 0));
  EXPECT_EQ(3.0, Binomial(3, 1));
  EXPECT_EQ(3.0, Binomial(3, 2));
  EXPECT_EQ(1.0, Binomial(3, 3));
}

// Test a 4th order polynomial.
TEST(BinomialTest, Quartic) {
  EXPECT_EQ(1.0, Binomial(4, 0));
  EXPECT_EQ(4.0, Binomial(4, 1));
  EXPECT_EQ(6.0, Binomial(4, 2));
  EXPECT_EQ(4.0, Binomial(4, 3));
  EXPECT_EQ(1.0, Binomial(4, 4));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
