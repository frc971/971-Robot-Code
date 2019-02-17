#ifndef CORE_TIME_H_
#define CORE_TIME_H_

#include <stdint.h>

// This whole file is deprecated. Use //aos/time instead.

#ifdef __cplusplus
extern "C"
{
#endif

// Returns the current number of nanoseconds. This will wrap naturally.
uint32_t nanos(void);

// Returns the current number of microseconds. This will wrap naturally.
uint32_t micros(void);

// Returns the current number of milliseconds. This will wrap naturally.
uint32_t millis(void);

// Delays for the specified number of milliseconds.
void delay(uint32_t ms);

// Delays for the specified number of milliseconds relative to the starting
// microsecond value.  Returns the ending microsecond value.
//
// This lets you hit an accurate absolute loop time with variable length
// calculations in the loop. It will handle wrapping correctly.
uint32_t delay_from(uint32_t start, uint32_t ms);

// Ways to add and subtract times without weird promotions to make sure wrapping
// works correctly.
static inline uint32_t time_add(uint32_t a, uint32_t b) { return a + b; }
static inline uint32_t time_subtract(uint32_t a, uint32_t b) { return a - b; }

// Returns 1 iff a is "after" b. This will do as well as it can in the presence
// of wrapping, but comparing times more than a half wrap apart will return the
// "wrong" answer.
static inline int time_after(uint32_t a, uint32_t b) {
  return (a - b - 1) < (uint32_t)(1u << 31);
}

#ifdef __cplusplus
}
#endif

#endif  // CORE_TIME_H_
