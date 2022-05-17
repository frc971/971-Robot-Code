#include "aos/util/phased_loop.h"

#include "aos/time/time.h"
#include "gtest/gtest.h"

namespace aos {
namespace time {
namespace testing {

using ::std::chrono::milliseconds;
using ::std::chrono::nanoseconds;

typedef ::testing::Test PhasedLoopTest;
typedef PhasedLoopTest PhasedLoopDeathTest;

monotonic_clock::time_point InMs(int ms) {
  return monotonic_clock::time_point(::std::chrono::milliseconds(ms));
}

TEST_F(PhasedLoopTest, Reset) {
  {
    PhasedLoop loop(milliseconds(100), monotonic_clock::epoch(),
                    milliseconds(0));

    loop.Reset(monotonic_clock::epoch());
    EXPECT_EQ(InMs(0), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(monotonic_clock::epoch()));
    EXPECT_EQ(InMs(100), loop.sleep_time());

    loop.Reset(InMs(99));
    EXPECT_EQ(InMs(0), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(99)));
    EXPECT_EQ(InMs(100), loop.sleep_time());

    loop.Reset(InMs(100));
    EXPECT_EQ(InMs(100), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(199)));
    EXPECT_EQ(InMs(200), loop.sleep_time());

    loop.Reset(InMs(101));
    EXPECT_EQ(InMs(100), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(101)));
    EXPECT_EQ(InMs(200), loop.sleep_time());
  }
  {
    PhasedLoop loop(milliseconds(100), monotonic_clock::epoch(),
                    milliseconds(1));
    loop.Reset(monotonic_clock::epoch());
    EXPECT_EQ(InMs(-99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(monotonic_clock::epoch()));
    EXPECT_EQ(InMs(1), loop.sleep_time());
  }
  {
    PhasedLoop loop(milliseconds(100), monotonic_clock::epoch(),
                    milliseconds(99));

    loop.Reset(monotonic_clock::epoch());
    EXPECT_EQ(InMs(-1), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(monotonic_clock::epoch()));
    EXPECT_EQ(InMs(99), loop.sleep_time());

    loop.Reset(InMs(98));
    EXPECT_EQ(InMs(-1), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(98)));
    EXPECT_EQ(InMs(99), loop.sleep_time());

    loop.Reset(InMs(99));
    EXPECT_EQ(InMs(99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(99)));
    EXPECT_EQ(InMs(199), loop.sleep_time());

    loop.Reset(InMs(100));
    EXPECT_EQ(InMs(99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(100)));
    EXPECT_EQ(InMs(199), loop.sleep_time());
  }
}

TEST_F(PhasedLoopTest, Iterate) {
  {
    PhasedLoop loop(milliseconds(100), monotonic_clock::epoch(),
                    milliseconds(99));
    loop.Reset(monotonic_clock::epoch());
    EXPECT_EQ(1, loop.Iterate(monotonic_clock::epoch()));
    EXPECT_EQ(InMs(99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(100)));
    EXPECT_EQ(InMs(199), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(InMs(100)));
    EXPECT_EQ(InMs(199), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(InMs(101)));
    EXPECT_EQ(InMs(199), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(InMs(198)));
    EXPECT_EQ(InMs(199), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(199)));
    EXPECT_EQ(InMs(299), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(300)));
    EXPECT_EQ(InMs(399), loop.sleep_time());
    EXPECT_EQ(3, loop.Iterate(InMs(600)));
    EXPECT_EQ(InMs(699), loop.sleep_time());
  }
  {
    PhasedLoop loop(milliseconds(100), monotonic_clock::epoch(),
                    milliseconds(1));
    loop.Reset(monotonic_clock::epoch());
    EXPECT_EQ(1, loop.Iterate(monotonic_clock::epoch()));
    EXPECT_EQ(InMs(1), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(100)));
    EXPECT_EQ(InMs(101), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(InMs(100)));
    EXPECT_EQ(InMs(101), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(103)));
    EXPECT_EQ(InMs(201), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(InMs(198)));
    EXPECT_EQ(InMs(201), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(InMs(200)));
    EXPECT_EQ(InMs(201), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(InMs(201)));
    EXPECT_EQ(InMs(301), loop.sleep_time());
    EXPECT_EQ(3, loop.Iterate(InMs(600)));
    EXPECT_EQ(InMs(601), loop.sleep_time());
  }
}

// Makes sure that everything works correctly when crossing zero.
// This seems like a rare case at first, but starting from zero needs to
// work, which means negatives should too.
TEST_F(PhasedLoopTest, CrossingZero) {
  PhasedLoop loop(milliseconds(100), monotonic_clock::epoch(), milliseconds(1));
  loop.Reset(InMs(-1000));
  EXPECT_EQ(InMs(-1099), loop.sleep_time());
  EXPECT_EQ(9, loop.Iterate(InMs(-250)));
  EXPECT_EQ(InMs(-199), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(InMs(-199)));
  EXPECT_EQ(InMs(-99), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(InMs(-90)));
  EXPECT_EQ(InMs(1), loop.sleep_time());
  EXPECT_EQ(0, loop.Iterate(InMs(0)));
  EXPECT_EQ(InMs(1), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(InMs(1)));
  EXPECT_EQ(InMs(101), loop.sleep_time());

  EXPECT_EQ(0, loop.Iterate(InMs(2)));
  EXPECT_EQ(InMs(101), loop.sleep_time());

  EXPECT_EQ(-2, loop.Iterate(InMs(-101)));
  EXPECT_EQ(InMs(-99), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(InMs(-99)));
  EXPECT_EQ(InMs(1), loop.sleep_time());

  EXPECT_EQ(0, loop.Iterate(InMs(-99)));
  EXPECT_EQ(InMs(1), loop.sleep_time());
}

// Tests OffsetFromIntervalAndTime for various edge conditions.
TEST_F(PhasedLoopTest, OffsetFromIntervalAndTimeTest) {
  PhasedLoop loop(milliseconds(1000), monotonic_clock::epoch(),
                  milliseconds(300));

  EXPECT_EQ(milliseconds(1),
            loop.OffsetFromIntervalAndTime(milliseconds(1000), InMs(1001)));

  EXPECT_EQ(milliseconds(0),
            loop.OffsetFromIntervalAndTime(milliseconds(1000), InMs(1000)));

  EXPECT_EQ(milliseconds(0),
            loop.OffsetFromIntervalAndTime(milliseconds(1000), InMs(0)));

  EXPECT_EQ(milliseconds(999),
            loop.OffsetFromIntervalAndTime(milliseconds(1000), InMs(-1)));

  EXPECT_EQ(milliseconds(7),
            loop.OffsetFromIntervalAndTime(milliseconds(1000), InMs(19115007)));

  EXPECT_EQ(milliseconds(7), loop.OffsetFromIntervalAndTime(milliseconds(1000),
                                                            InMs(-19115993)));
}

// Tests that passing invalid values to the constructor dies correctly.
TEST_F(PhasedLoopDeathTest, InvalidValues) {
  EXPECT_DEATH(
      PhasedLoop(milliseconds(1), monotonic_clock::epoch(), milliseconds(2)),
      ".*offset < interval.*");
  EXPECT_DEATH(
      PhasedLoop(milliseconds(1), monotonic_clock::epoch(), milliseconds(1)),
      ".*offset < interval.*");
  EXPECT_DEATH(
      PhasedLoop(milliseconds(1), monotonic_clock::epoch(), milliseconds(-1)),
      ".*offset >= monotonic_clock::duration\\(0\\).*");
  EXPECT_DEATH(
      PhasedLoop(milliseconds(0), monotonic_clock::epoch(), milliseconds(0)),
      ".*interval > monotonic_clock::duration\\(0\\).*");
}

// Tests that every single value within two intervals of 0 works.
// This is good at finding edge cases in the rounding.
TEST_F(PhasedLoopTest, SweepingZero) {
  for (int i = -30; i < -20; ++i) {
    PhasedLoop loop(nanoseconds(20),
                    monotonic_clock::epoch() - nanoseconds(30));
    EXPECT_EQ(1, loop.Iterate(monotonic_clock::epoch() + nanoseconds(i)));
  }
  for (int i = -20; i < 0; ++i) {
    PhasedLoop loop(nanoseconds(20),
                    monotonic_clock::epoch() - nanoseconds(30));
    EXPECT_EQ(2, loop.Iterate(monotonic_clock::epoch() + nanoseconds(i)));
  }
  for (int i = 0; i < 20; ++i) {
    PhasedLoop loop(nanoseconds(20),
                    monotonic_clock::epoch() - nanoseconds(30));
    EXPECT_EQ(3, loop.Iterate(monotonic_clock::epoch() + nanoseconds(i)));
  }
  for (int i = 20; i < 30; ++i) {
    PhasedLoop loop(nanoseconds(20),
                    monotonic_clock::epoch() - nanoseconds(30));
    EXPECT_EQ(4, loop.Iterate(monotonic_clock::epoch() + nanoseconds(i)));
  }
}

// Tests that the phased loop is correctly adjusting when the offset is
// decremented multiple times.
TEST_F(PhasedLoopTest, DecrementingOffset) {
  constexpr int kCount = 5;
  constexpr int kIterations = 10;
  const auto kOffset = milliseconds(400);
  const auto kInterval = milliseconds(1000);
  const auto kAllIterationsInterval = kInterval * kIterations;

  PhasedLoop loop(kInterval, monotonic_clock::epoch(), kOffset);
  auto last_time = monotonic_clock::epoch() + kOffset + (kInterval * 3);
  ASSERT_EQ(5, loop.Iterate(last_time));
  for (int i = 1; i < kCount; i++) {
    const auto offset = kOffset - milliseconds(i);
    loop.set_interval_and_offset(kInterval, offset);
    const auto next_time = last_time - milliseconds(1) + kAllIterationsInterval;
    EXPECT_EQ(kIterations, loop.Iterate(next_time));
    last_time = next_time;
  }
}

// Tests that the phased loop is correctly adjusting when the offset is
// changed to 0.
TEST_F(PhasedLoopTest, ChangingOffset) {
  const auto kOffset = milliseconds(900);
  const auto kInterval = milliseconds(1000);
  PhasedLoop loop(kInterval, monotonic_clock::epoch(), kOffset);
  const auto last_time = monotonic_clock::epoch() + kOffset + (kInterval * 3);
  ASSERT_EQ(5, loop.Iterate(last_time));
  loop.set_interval_and_offset(kInterval, milliseconds(0));
  EXPECT_EQ(4, loop.Iterate((last_time - kOffset) + (kInterval * 4)));
}

}  // namespace testing
}  // namespace time
}  // namespace aos
