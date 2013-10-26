#ifndef AOS_COMMON_UTIL_WRAPPING_COUNTER_H_
#define AOS_COMMON_UTIL_WRAPPING_COUNTER_H_

#include <stdint.h>

namespace aos {
namespace util {

// Deals correctly with 1-byte counters which wrap.
// This is only possible if the counter never wraps twice between Update calls.
// It will also fail if the counter ever goes down (that will be interpreted as
// +255 instead of -1, for example).
class WrappingCounter {
 public:
  WrappingCounter(int32_t initial_count = 0);

  // Updates the internal counter with a new raw value.
  // Returns count() for convenience.
  int32_t Update(uint8_t current);

  // Resets the actual count to value.
  void Reset(int32_t value = 0) { count_ = value; }

  int32_t count() const { return count_; }

 private:
  int32_t count_;
  uint8_t last_count_;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_WRAPPING_COUNTER_H_
