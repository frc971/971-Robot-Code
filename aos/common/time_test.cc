#include "aos/common/time.h"

#include <thread>

#include "gtest/gtest.h"

#include "aos/common/macros.h"
#include "aos/common/util/death_test_log_implementation.h"

namespace aos {
namespace time {
namespace testing {


TEST(TimeTest, FromRate) {
  EXPECT_EQ(::std::chrono::milliseconds(10), FromRate(100));
}

// Test the monotonic_clock and sleep_until functions.
TEST(TimeTest, MonotonicClockSleepAndNow) {
  monotonic_clock::time_point start = monotonic_clock::now();
  const auto kSleepTime = ::std::chrono::milliseconds(500);
  ::std::this_thread::sleep_until(start + kSleepTime);
  monotonic_clock::time_point end = monotonic_clock::now();
  EXPECT_GE(end - start, kSleepTime);
  EXPECT_LT(end - start, kSleepTime + ::std::chrono::milliseconds(200));
}

}  // namespace testing
}  // namespace time
}  // namespace aos
