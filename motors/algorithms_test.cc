#include "motors/algorithms.h"

#include <inttypes.h>
#include <cmath>

#include "gtest/gtest.h"

namespace frc971 {
namespace motors {
namespace testing {

class BalanceReadingsTest : public ::testing::Test {
 protected:
  void CheckReadingsResult(const ReadingsToBalance &to_balance) {
    ASSERT_GE((to_balance.weights[0] > 0) + (to_balance.weights[1] > 0) +
                  (to_balance.weights[2] > 0),
              2)
        << "Need at least 2 readings";
    ASSERT_GE(to_balance.weights[0], 0);
    ASSERT_GE(to_balance.weights[1], 0);
    ASSERT_GE(to_balance.weights[2], 0);

    const BalancedReadings result = BalanceReadings(to_balance);

    {
      const auto sum =
          result.readings[0] + result.readings[1] + result.readings[2];
      EXPECT_GE(sum, -2);
      EXPECT_LE(sum, 2);
    }

    if (to_balance.weights[0] == 0) {
      const double averages[3] = {
          0, static_cast<double>(to_balance.sums[1]) / to_balance.weights[1],
          static_cast<double>(to_balance.sums[2]) / to_balance.weights[2]};
      EXPECT_LE(::std::abs((averages[1] - averages[2]) -
                           (result.readings[1] - result.readings[2])),
                0.5);
    } else if (to_balance.weights[1] == 0) {
      const double averages[3] = {
          static_cast<double>(to_balance.sums[0]) / to_balance.weights[0], 0,
          static_cast<double>(to_balance.sums[2]) / to_balance.weights[2]};
      EXPECT_LE(::std::abs((averages[0] - averages[2]) -
                           (result.readings[0] - result.readings[2])),
                0.5);
    } else if (to_balance.weights[2] == 0) {
      const double averages[3] = {
          static_cast<double>(to_balance.sums[0]) / to_balance.weights[0],
          static_cast<double>(to_balance.sums[1]) / to_balance.weights[1], 0};
      EXPECT_LE(::std::abs((averages[0] - averages[1]) -
                           (result.readings[0] - result.readings[1])),
                0.5);
    } else {
      const double averages[3] = {
          static_cast<double>(to_balance.sums[0]) / to_balance.weights[0],
          static_cast<double>(to_balance.sums[1]) / to_balance.weights[1],
          static_cast<double>(to_balance.sums[2]) / to_balance.weights[2]};

      const double middle = (averages[0] + averages[1] + averages[2]) / 3;
      const double average_distances[3] = {
          ::std::abs(averages[0] - middle - result.readings[0]),
          ::std::abs(averages[1] - middle - result.readings[1]),
          ::std::abs(averages[2] - middle - result.readings[2])};
      // distances[0]/distances[1] = weights[1]/weights[0]
      // distances[0]*weights[0]/weights[1] = distances[1]
      EXPECT_LE(::std::abs(average_distances[0] *
                               static_cast<double>(to_balance.weights[0]) /
                               static_cast<double>(to_balance.weights[1]) -
                           average_distances[1]),
                0.01);
      EXPECT_LE(::std::abs(average_distances[2] *
                               static_cast<double>(to_balance.weights[2]) /
                               static_cast<double>(to_balance.weights[1]) -
                           average_distances[1]),
                0.01);
      EXPECT_LE(::std::abs(average_distances[0] *
                               static_cast<double>(to_balance.weights[0]) /
                               static_cast<double>(to_balance.weights[2]) -
                           average_distances[2]),
                0.01);
    }
  }
};

TEST_F(BalanceReadingsTest, Basic) {
  CheckReadingsResult({{50, 50, 50}, {1, 1, 1}});
  CheckReadingsResult({{50, 50, 0}, {1, 1, 0}});
  CheckReadingsResult({{50, 0, 50}, {1, 0, 1}});
  CheckReadingsResult({{0, 50, 50}, {0, 1, 1}});
  CheckReadingsResult({{0, 50, 100}, {0, 1, 2}});
  CheckReadingsResult({{100, 50, 50}, {2, 1, 1}});
  CheckReadingsResult({{100, 100, 50}, {2, 2, 1}});

  CheckReadingsResult({{150, 50, 50}, {1, 1, 1}});
  CheckReadingsResult({{150, 50, 50}, {2, 2, 2}});
  CheckReadingsResult({{3424, 5013, 3424}, {2, 2, 2}});
}

}  // namespace testing
}  // namespace motors
}  // namespace frc971
