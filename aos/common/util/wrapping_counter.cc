#include "aos/common/util/wrapping_counter.h"

namespace aos {
namespace util {

WrappingCounter::WrappingCounter(int32_t initial_count)
    : count_(initial_count), last_count_(0) {}

int32_t WrappingCounter::Update(uint8_t current) {
  if (last_count_ > current) {
    count_ += 0x100;
  }
  count_ = (count_ & 0xffffff00) | current;
  last_count_ = current;
  return count_;
}

}  // namespace util
}  // namespace aos
