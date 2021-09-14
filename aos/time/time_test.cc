#include "aos/time/time.h"

#include <thread>

#include "gtest/gtest.h"
#include "glog/logging.h"

#include "aos/macros.h"
#include "aos/util/death_test_log_implementation.h"

namespace aos {
namespace time {
namespace testing {

namespace chrono = std::chrono;

TEST(TimeTest, FromRate) { EXPECT_EQ(chrono::milliseconds(10), FromRate(100)); }

// Test the monotonic_clock and sleep_until functions.
TEST(TimeTest, MonotonicClockSleepAndNow) {
  monotonic_clock::time_point start = monotonic_clock::now();
  const auto kSleepTime = chrono::milliseconds(500);
  ::std::this_thread::sleep_until(start + kSleepTime);
  monotonic_clock::time_point end = monotonic_clock::now();
  EXPECT_GE(end - start, kSleepTime);
  EXPECT_LT(end - start, kSleepTime + chrono::milliseconds(200));
}

// Test to_timespec for a duration.
TEST(TimeTest, DurationToTimespec) {
  struct timespec pos_time = to_timespec(chrono::milliseconds(56262));
  EXPECT_EQ(pos_time.tv_sec, 56);
  EXPECT_EQ(pos_time.tv_nsec, 262000000);

  struct timespec neg_time = to_timespec(chrono::milliseconds(-56262));
  EXPECT_EQ(neg_time.tv_sec, -56);
  EXPECT_EQ(neg_time.tv_nsec, -262000000);
}

// Test to_timespec for a time_point.
TEST(TimeTest, TimePointToTimespec) {
  struct timespec pos_time =
      to_timespec(::aos::monotonic_clock::epoch() + chrono::seconds(1432423));
  EXPECT_EQ(pos_time.tv_sec, 1432423);
  EXPECT_EQ(pos_time.tv_nsec, 0);

  struct timespec neg_time =
      to_timespec(::aos::monotonic_clock::epoch() - chrono::seconds(1432423));
  EXPECT_EQ(neg_time.tv_sec, -1432423);
  EXPECT_EQ(neg_time.tv_nsec, 0);
}

// Tests from_timeval.
TEST(TimeTest, TimevalToTimePoint) {
  struct timeval pos_time;
  pos_time.tv_sec = 1432423;
  pos_time.tv_usec = 0;
  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1432423),
            from_timeval(pos_time));

  struct timeval neg_time;
  neg_time.tv_sec = -1432423;
  neg_time.tv_usec = 0;
  EXPECT_EQ(::aos::monotonic_clock::epoch() - chrono::seconds(1432423),
            from_timeval(neg_time));
}

// Test that << works with numbers with leading 0's.
TEST(TimeTest, OperatorStream) {
  const monotonic_clock::time_point t = monotonic_clock::epoch() +
                                        chrono::seconds(1432423) +
                                        chrono::milliseconds(15);

  // And confirm that the stream's settings are restored by adding a random
  // number afterwords.
  std::stringstream s;
  s << t << " and number " << 123;

  EXPECT_EQ(s.str(), "1432423.015000000sec and number 123");
}

// Test that << works with negative numbers.
TEST(TimeTest, OperatorStreamNegative) {
  {
    const monotonic_clock::time_point t = monotonic_clock::epoch() -
                                          chrono::seconds(14) +
                                          chrono::milliseconds(915);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-13.085000000sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
  {
    const monotonic_clock::time_point t =
        monotonic_clock::epoch() - chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-0.000000001sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
  {
    const monotonic_clock::time_point t =
        monotonic_clock::epoch() - chrono::seconds(1) - chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-1.000000001sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
  {
    const monotonic_clock::time_point t =
        monotonic_clock::epoch() - chrono::seconds(2) - chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-2.000000001sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
}

// Test that << works with min_time.
TEST(TimeTest, OperatorStreamMinTime) {
  const monotonic_clock::time_point t = monotonic_clock::min_time;

  std::stringstream s;
  s << t;

  EXPECT_EQ(s.str(), "-9223372036.854775808sec");
  EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
}

// Test that << works with the epoch on the realtime clock.
TEST(TimeTest, OperatorStreamRealtimeEpoch) {
  const realtime_clock::time_point t = realtime_clock::epoch();

  std::stringstream s;
  s << t;

  EXPECT_EQ(s.str(), "1970-01-01_00-00-00.000000000");
  EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
}

// Test that << works with positive time on the realtime clock.
TEST(TimeTest, OperatorStreamRealtimePositive) {
  const realtime_clock::time_point t =
      realtime_clock::epoch() + std::chrono::hours(5 * 24) +
      std::chrono::seconds(11) + std::chrono::milliseconds(5);

  std::stringstream s;
  s << t;

  EXPECT_EQ(s.str(), "1970-01-06_00-00-11.005000000");
  EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
}

// Test that << works with negative time on the realtime clock.
TEST(TimeTest, OperatorStreamRealtimeNegative) {
  {
    const realtime_clock::time_point t =
        realtime_clock::epoch() - std::chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-31_23-59-59.999999999");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
  {
    const realtime_clock::time_point t =
        realtime_clock::epoch() - std::chrono::nanoseconds(999999999);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-31_23-59-59.000000001");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
  {
    const realtime_clock::time_point t = realtime_clock::epoch() -
                                         std::chrono::seconds(1) -
                                         std::chrono::nanoseconds(999999999);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-31_23-59-58.000000001");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
  {
    const realtime_clock::time_point t =
        realtime_clock::epoch() - std::chrono::hours(5 * 24) +
        std::chrono::seconds(11) - std::chrono::milliseconds(5);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-27_00-00-10.995000000");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }

  {
    const realtime_clock::time_point t = realtime_clock::min_time;

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1677-09-21_00-12-43.145224192");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }

  {
    const realtime_clock::time_point t =
        realtime_clock::min_time + std::chrono::nanoseconds(999999999);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1677-09-21_00-12-44.145224191");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
}

}  // namespace testing
}  // namespace time
}  // namespace aos
