#include "motors/algorithms.h"

namespace frc971 {
namespace motors {

BalancedReadings BalanceReadings(const ReadingsToBalance to_balance) {
  // TODO(Brian): Get rid of the floating point divides.
  BalancedReadings result;
  if (to_balance.weights[0] == 0) {
    const float average1 = static_cast<float>(to_balance.sums[1]) /
                           static_cast<float>(to_balance.weights[1]);
    const float average2 = static_cast<float>(to_balance.sums[2]) /
                           static_cast<float>(to_balance.weights[2]);
    result.readings[0] = -(average1 + average2);
    result.readings[1] = average1;
    result.readings[2] = average2;
  } else if (to_balance.weights[1] == 0) {
    const float average0 = static_cast<float>(to_balance.sums[0]) /
                           static_cast<float>(to_balance.weights[0]);
    const float average2 = static_cast<float>(to_balance.sums[2]) /
                           static_cast<float>(to_balance.weights[2]);
    result.readings[0] = average0;
    result.readings[1] = -(average0 + average2);
    result.readings[2] = average2;
  } else if (to_balance.weights[2] == 0) {
    const float average0 = static_cast<float>(to_balance.sums[0]) /
                           static_cast<float>(to_balance.weights[0]);
    const float average1 = static_cast<float>(to_balance.sums[1]) /
                           static_cast<float>(to_balance.weights[1]);
    result.readings[0] = average0;
    result.readings[1] = average1;
    result.readings[2] = -(average0 + average1);
  } else {
    const float average0 = static_cast<float>(to_balance.sums[0]) /
                           static_cast<float>(to_balance.weights[0]);
    const float average1 = static_cast<float>(to_balance.sums[1]) /
                           static_cast<float>(to_balance.weights[1]);
    const float average2 = static_cast<float>(to_balance.sums[2]) /
                           static_cast<float>(to_balance.weights[2]);
    const float offset = (average0 + average1 + average2) / -3;
    result.readings[0] = average0 + offset;
    result.readings[1] = average1 + offset;
    result.readings[2] = average2 + offset;
  }

  return result;
}

}  // namespace motors
}  // namespace frc971
