#ifndef MOTORS_ALGORITHMS_H_
#define MOTORS_ALGORITHMS_H_

#include <stdint.h>

namespace frc971 {
namespace salsa {

struct ReadingsToBalance {
  // Adds a single reading at index.
  void Add(int index, int32_t value) {
    sums[index] += value;
    ++weights[index];
  }

  int32_t sums[3];
  int weights[3];
};

struct BalancedReadings {
  float readings[3];
};

// Returns three readings which add up to 0 and are the same distances apart as
// the input ones (by weight). The distances between the averages of the inputs
// and the corresponding outputs will be inversely proportional to the weights.
BalancedReadings BalanceReadings(ReadingsToBalance to_balance);

}  // namespace salsa
}  // namespace frc971

#endif  // MOTORS_ALGORITHMS_H_
