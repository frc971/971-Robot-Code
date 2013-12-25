#ifndef CAPE_ENCODER_H_
#define CAPE_ENCODER_H_

#include <stdint.h>
#include <limits.h>

#include <STM32F2XX.h>

void encoder_init(void);

// Updates a signed 32-bit counter with a new 16-bit value. Assumes that the
// value will not more than half-wrap between updates.
// new is 32 bits so it doesn't have to get masked, but the value passed in must
// be <= UINT16_MAX.
// Useful for 16-bit encoder counters.
static inline void counter_update_s32_u16(int32_t *restrict counter,
                                          uint32_t new) {
  static const uint16_t kHalf = 0xFFFF / 2;
  uint16_t old = *counter & 0xFFFF;
  int32_t counter_top = *counter ^ old;
  int32_t delta = (int32_t)old - (int32_t)new;
  int32_t new_counter;
  if (__builtin_expect(delta < -kHalf, 0)) {
    new_counter = (counter_top - 0x10000) ^ 0xFFFF;
  } else if (__builtin_expect(delta > kHalf, 0)) {
    new_counter = counter_top + 0x10000;
  } else {
    new_counter = counter_top;
  }
  *counter = new_counter | new;
}

// Updates an unsigned 64-bit counter with a new 16-bit value. Assumes that the
// value will not wrap more than once between updates.
// new is 32 bits so it doesn't have to get masked, but the value passed in must
// be <= UINT16_MAX.
// Useful for 16-bit timers being used for absolute timings.
static inline void counter_update_u64_u16(uint64_t *restrict counter,
                                          uint32_t new) {
  uint16_t old = *counter & 0xFFFF;
  int64_t counter_top = *counter ^ old;
  int64_t new_counter;
  if (__builtin_expect(new < old, 0)) {
    new_counter = counter_top + 0x10000;
  } else {
    new_counter = counter_top;
  }
  *counter = new_counter | new;
}

// number is the 0-indexed number on the silkscreen
static inline int32_t encoder_read(int number) {
  static int32_t value0, value6, value7;
  extern volatile int32_t encoder1_value, encoder3_value, encoder4_value;
  switch (number) {
    case 0:
      counter_update_s32_u16(&value0, TIM8->CNT);
      return value0;
    case 1:
      return encoder1_value;
    case 2:
      return TIM5->CNT;
    case 3:
      return encoder3_value;
    case 4:
      return encoder4_value;
    case 5:
      return TIM2->CNT;
    case 6:
      counter_update_s32_u16(&value6, TIM3->CNT);
      return value6;
    case 7:
      counter_update_s32_u16(&value7, TIM4->CNT);
      return value7;
    default:
      return INT32_MAX;
  }
}

#endif  // CAPE_ENCODER_H_
