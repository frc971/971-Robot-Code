#include "aos/time/time.h"

#include <thread>

#include "gtest/gtest.h"

#include "aos/macros.h"
#include "aos/util/death_test_log_implementation.h"

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

// Test to_timespec for a duration.
TEST(TimeTest, DurationToTimespec) {
  struct timespec pos_time = to_timespec(::std::chrono::milliseconds(56262));
  EXPECT_EQ(pos_time.tv_sec, 56);
  EXPECT_EQ(pos_time.tv_nsec, 262000000);

  struct timespec neg_time = to_timespec(::std::chrono::milliseconds(-56262));
  EXPECT_EQ(neg_time.tv_sec, -56);
  EXPECT_EQ(neg_time.tv_nsec, -262000000);
}

// Test to_timespec for a time_point.
TEST(TimeTest, TimePointToTimespec) {
  struct timespec pos_time = to_timespec(::aos::monotonic_clock::epoch() +
                                     ::std::chrono::seconds(1432423));
  EXPECT_EQ(pos_time.tv_sec, 1432423);
  EXPECT_EQ(pos_time.tv_nsec, 0);

  struct timespec neg_time = to_timespec(::aos::monotonic_clock::epoch() -
                                     ::std::chrono::seconds(1432423));
  EXPECT_EQ(neg_time.tv_sec, -1432423);
  EXPECT_EQ(neg_time.tv_nsec, 0);
}

}  // namespace testing
}  // namespace time
}  // namespace aos
