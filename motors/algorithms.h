#ifndef MOTORS_ALGORITHMS_H_
#define MOTORS_ALGORITHMS_H_

#include <stdint.h>
#include <array>

namespace frc971 {
namespace motors {

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

inline BalancedReadings BalanceSimpleReadings(
    const ::std::array<float, 3> readings) {
  float offset = 0;
  for (int i = 0; i < 3; ++i) {
    offset += readings[i];
  }

  offset = offset / 3.0f;

  BalancedReadings r;
  for (int i = 0; i < 3; ++i) {
    r.readings[i] = static_cast<float>(readings[i]) - offset;
  }
  return r;
}

inline BalancedReadings BalanceSimpleReadings(const uint16_t readings[3]) {
  float offset = 0;
  for (int i = 0; i < 3; ++i) {
    offset += static_cast<float>(readings[i]);
  }

  BalancedReadings r;
  for (int i = 0; i < 3; ++i) {
    r.readings[i] = static_cast<float>(readings[i]) + offset / -3;
  }
  return r;
}

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_ALGORITHMS_H_
