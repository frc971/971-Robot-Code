#include "aos/common/time.h"

#include <thread>

#include "gtest/gtest.h"

#include "aos/common/macros.h"
#include "aos/common/util/death_test_log_implementation.h"

namespace aos {
namespace time {
namespace testing {

TEST(TimeTest, timespecConversions) {
  timespec start{1234, 5678};  // NOLINT
  Time time(start);
  EXPECT_EQ(start.tv_sec, static_cast<time_t>(time.sec()));
  EXPECT_EQ(start.tv_nsec, time.nsec());
  timespec end = time.ToTimespec();
  EXPECT_EQ(start.tv_sec, end.tv_sec);
  EXPECT_EQ(start.tv_nsec, end.tv_nsec);
}

TEST(TimeTest, timevalConversions) {
  timeval start{1234, 5678};  // NOLINT
  Time time(start);
  EXPECT_EQ(start.tv_sec, static_cast<long>(time.sec()));
  EXPECT_EQ(start.tv_usec, time.nsec() / Time::kNSecInUSec);
  timeval end = time.ToTimeval();
  EXPECT_EQ(start.tv_sec, end.tv_sec);
  EXPECT_EQ(start.tv_usec, end.tv_usec);
}

TEST(TimeDeathTest, ConstructorChecking) {
  logging::Init();
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        Time(0, -1);
      },
      ".*0 <= nsec\\(-1\\) < 10+ .*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        Time(0, Time::kNSecInSec);
      },
      ".*0 <= nsec\\(10+\\) < 10+ .*");
}

// It's kind of hard not to test Now and SleepFor at the same time.
TEST(TimeTest, NowAndSleepFor) {
  // without this, it tends to fail the first time (ends up sleeping for way
  // longer than it should the second time, where it actually matters)
  SleepFor(Time(0, Time::kNSecInSec / 10));
  Time start = Time::Now();
  static constexpr Time kSleepTime = Time(0, Time::kNSecInSec * 2 / 10);
  SleepFor(kSleepTime);
  Time difference = Time::Now() - start;
  EXPECT_GE(difference, kSleepTime);
  EXPECT_LT(difference, kSleepTime + Time(0, Time::kNSecInSec / 100));
}

TEST(TimeTest, AbsoluteSleep) {
  Time start = Time::Now();
  SleepFor(Time(0, Time::kNSecInSec / 10));
  static constexpr Time kSleepTime = Time(0, Time::kNSecInSec * 2 / 10);
  SleepUntil(start + kSleepTime);
  Time difference = Time::Now() - start;
  EXPECT_GE(difference, kSleepTime);
  EXPECT_LT(difference, kSleepTime + Time(0, Time::kNSecInSec / 100));
}

TEST(TimeTest, Addition) {
  Time t(54, 500);
  EXPECT_EQ(MACRO_DARG(Time(54, 5500)), t + MACRO_DARG(Time(0, 5000)));
  EXPECT_EQ(MACRO_DARG(Time(56, 500)), t + MACRO_DARG(Time(2, 0)));
  EXPECT_EQ(MACRO_DARG(Time(57, 6500)), t + MACRO_DARG(Time(3, 6000)));
  EXPECT_EQ(MACRO_DARG(Time(50, 300)),
            t + MACRO_DARG(Time(-5, Time::kNSecInSec - 200)));
  EXPECT_EQ(Time(-46, 500), t + Time(-100, 0));
  EXPECT_EQ(Time(-47, Time::kNSecInSec - 500),
            Time(-101, Time::kNSecInSec - 1000) + t);
}
TEST(TimeTest, Subtraction) {
  Time t(54, 500);
  EXPECT_EQ(MACRO_DARG(Time(54, 300)), t - MACRO_DARG(Time(0, 200)));
  EXPECT_EQ(MACRO_DARG(Time(42, 500)), t - MACRO_DARG(Time(12, 0)));
  EXPECT_EQ(MACRO_DARG(Time(50, 100)), t - MACRO_DARG(Time(4, 400)));
  EXPECT_EQ(MACRO_DARG(Time(53, 600)),
            t - MACRO_DARG(Time(0, Time::kNSecInSec - 100)));
  EXPECT_EQ(MACRO_DARG(Time(55, 800)),
            t - MACRO_DARG(Time(-2, Time::kNSecInSec - 300)));
  EXPECT_EQ(Time(54, 5500), t - Time(-1, Time::kNSecInSec - 5000));
  EXPECT_EQ(Time(-50, Time::kNSecInSec - 300),
            Time(5, 200) - t);
}

TEST(TimeTest, Multiplication) {
  Time t(54, Time::kNSecInSec / 3);
  EXPECT_EQ(MACRO_DARG(Time(108, Time::kNSecInSec / 3 * 2)), t * 2);
  EXPECT_EQ(MACRO_DARG(Time(271, Time::kNSecInSec / 3 * 2 - 1)), t * 5);
  EXPECT_EQ(Time(-109, Time::kNSecInSec / 3 + 1), t * -2);
  EXPECT_EQ(Time(-55, Time::kNSecInSec / 3 * 2 + 1), t * -1);
  EXPECT_EQ(Time(-218, Time::kNSecInSec / 3 * 2 + 2), (t * -1) * 4);
}
TEST(TimeTest, DivisionByInt) {
  EXPECT_EQ(Time(5, Time::kNSecInSec / 10 * 4 + 50), Time(54, 500) / 10);
  EXPECT_EQ(Time(2, Time::kNSecInSec / 4 * 3),
            Time(5, Time::kNSecInSec / 2) / 2);
  EXPECT_EQ(Time(-3, Time::kNSecInSec / 4 * 3),
            Time(-5, Time::kNSecInSec / 2) / 2);
}
TEST(TimeTest, DivisionByTime) {
  EXPECT_DOUBLE_EQ(2, Time(10, 0) / Time(5, 0));
  EXPECT_DOUBLE_EQ(9, Time(27, 0) / Time(3, 0));
  EXPECT_DOUBLE_EQ(9.25, Time(37, 0) / Time(4, 0));
  EXPECT_DOUBLE_EQ(5.25, Time(36, Time::kNSecInSec / 4 * 3) / Time(7, 0));
  EXPECT_DOUBLE_EQ(-5.25, Time(-37, Time::kNSecInSec / 4) / Time(7, 0));
  EXPECT_DOUBLE_EQ(-5.25, Time(36, Time::kNSecInSec / 4 * 3) / Time(-7, 0));
}

TEST(TimeTest, Negation) {
  EXPECT_EQ(Time(-5, 1234), -Time(4, Time::kNSecInSec - 1234));
  EXPECT_EQ(Time(5, Time::kNSecInSec * 2 / 3 + 1),
            -Time(-6, Time::kNSecInSec / 3));
}

TEST(TimeTest, Comparisons) {
  EXPECT_TRUE(Time(971, 254) > Time(971, 253));
  EXPECT_TRUE(Time(971, 254) >= Time(971, 253));
  EXPECT_TRUE(Time(971, 254) < Time(971, 255));
  EXPECT_TRUE(Time(971, 254) <= Time(971, 255));
  EXPECT_TRUE(Time(971, 254) >= Time(971, 253));
  EXPECT_TRUE(Time(971, 254) <= Time(971, 254));
  EXPECT_TRUE(Time(971, 254) >= Time(971, 254));
  EXPECT_TRUE(Time(972, 254) > Time(971, 254));
  EXPECT_TRUE(Time(971, 254) < Time(972, 254));

  EXPECT_TRUE(Time(-971, 254) > Time(-971, 253));
  EXPECT_TRUE(Time(-971, 254) >= Time(-971, 253));
  EXPECT_TRUE(Time(-971, 254) < Time(-971, 255));
  EXPECT_TRUE(Time(-971, 254) <= Time(-971, 255));
  EXPECT_TRUE(Time(-971, 254) >= Time(-971, 253));
  EXPECT_TRUE(Time(-971, 254) <= Time(-971, 254));
  EXPECT_TRUE(Time(-971, 254) >= Time(-971, 254));
  EXPECT_TRUE(Time(-972, 254) < Time(-971, 254));
  EXPECT_TRUE(Time(-971, 254) > Time(-972, 254));
}

TEST(TimeTest, Within) {
  EXPECT_TRUE(MACRO_DARG(Time(55, 5000).IsWithin(Time(55, 4900), 100)));
  EXPECT_FALSE(MACRO_DARG(Time(55, 5000).IsWithin(Time(55, 4900), 99)));
  EXPECT_TRUE(MACRO_DARG(Time(5, 0).IsWithin(Time(4, Time::kNSecInSec - 200),
                                             250)));
  EXPECT_TRUE(Time(-5, Time::kNSecInSec - 200).IsWithin(Time(-4, 0), 250));
  EXPECT_TRUE(Time(-5, 200).IsWithin(Time(-5, 0), 250));
}

TEST(TimeTest, Modulus) {
  EXPECT_EQ(MACRO_DARG(Time(0, Time::kNSecInSec / 10 * 2)),
            MACRO_DARG(Time(50, 0) % (Time::kNSecInSec / 10 * 3)));
  EXPECT_EQ(Time(-1, Time::kNSecInSec / 10 * 8),
            Time(-50, 0) % (Time::kNSecInSec / 10 * 3));
  EXPECT_EQ(Time(-1, Time::kNSecInSec / 10 * 8),
            Time(-50, 0) % (-Time::kNSecInSec / 10 * 3));
  EXPECT_EQ(Time(0, Time::kNSecInSec / 10 * 2),
            Time(50, 0) % (-Time::kNSecInSec / 10 * 3));
  EXPECT_EQ(Time(1, Time::kNSecInSec / 10),
            Time(60, Time::kNSecInSec / 10) % (Time::kNSecInSec / 10 * 12));
}

TEST(TimeTest, InSeconds) {
  EXPECT_EQ(MACRO_DARG(Time(2, Time::kNSecInSec / 100 * 55 - 1)),
            Time::InSeconds(2.55));
  EXPECT_EQ(MACRO_DARG(Time(-3, Time::kNSecInSec / 100 * 45)),
            Time::InSeconds(-2.55));
}

TEST(TimeTest, ToSeconds) {
  EXPECT_DOUBLE_EQ(13.23, Time::InSeconds(13.23).ToSeconds());
  EXPECT_NEAR(-13.23, Time::InSeconds(-13.23).ToSeconds(),
              1.0 / Time::kNSecInSec * 2);
}

TEST(TimeTest, InMS) {
  Time t = Time::InMS(254971);
  EXPECT_EQ(254, t.sec());
  EXPECT_EQ(971000000, t.nsec());

  Time t2 = Time::InMS(-254971);
  EXPECT_EQ(-255, t2.sec());
  EXPECT_EQ(Time::kNSecInSec - 971000000, t2.nsec());

  Time t3 = Time::InMS(-1000);
  EXPECT_EQ(-1, t3.sec());
  EXPECT_EQ(0, t3.nsec());

  Time t4 = Time::InMS(1000);
  EXPECT_EQ(1, t4.sec());
  EXPECT_EQ(0, t4.nsec());

  Time t5 = Time::InMS(1001);
  EXPECT_EQ(1, t5.sec());
  EXPECT_EQ(Time::kNSecInMSec, t5.nsec());

  Time t6 = Time::InMS(-1001);
  EXPECT_EQ(-2, t6.sec());
  EXPECT_EQ(Time::kNSecInSec - Time::kNSecInMSec, t6.nsec());

  Time t7 = Time::InMS(-999);
  EXPECT_EQ(-1, t7.sec());
  EXPECT_EQ(Time::kNSecInMSec, t7.nsec());

  Time t8 = Time::InMS(999);
  EXPECT_EQ(0, t8.sec());
  EXPECT_EQ(Time::kNSecInSec - Time::kNSecInMSec, t8.nsec());
}

TEST(TimeTest, ToMSec) {
  EXPECT_EQ(254971, Time(254, 971000000).ToMSec());
  EXPECT_EQ(-254971, Time(-255, Time::kNSecInSec - 971000000).ToMSec());
}

TEST(TimeTest, InNS) {
  Time t = Time::InNS(static_cast<int64_t>(973254111971ll));
  EXPECT_EQ(973, t.sec());
  EXPECT_EQ(254111971, t.nsec());

  Time t2 = Time::InNS(static_cast<int64_t>(-973254111971ll));
  EXPECT_EQ(-974, t2.sec());
  EXPECT_EQ(Time::kNSecInSec - 254111971, t2.nsec());
}

TEST(TimeTest, InUS) {
  Time t = Time::InUS(254111971);
  EXPECT_EQ(254, t.sec());
  EXPECT_EQ(111971000, t.nsec());

  Time t2 = Time::InUS(-254111971);
  EXPECT_EQ(-255, t2.sec());
  EXPECT_EQ(Time::kNSecInSec - 111971000, t2.nsec());
}

TEST(TimeTest, ToUSec) {
  EXPECT_EQ(254000971, Time(254, 971000).ToUSec());
  EXPECT_EQ(-254000971, Time(-255, Time::kNSecInSec - 971000).ToUSec());
}

TEST(TimeTest, Abs) {
  EXPECT_EQ(MACRO_DARG(Time(971, 1114)), MACRO_DARG(Time(971, 1114).abs()));
  EXPECT_EQ(MACRO_DARG(Time(253, Time::kNSecInSec * 0.3)),
            MACRO_DARG(Time(-254, Time::kNSecInSec * 0.7).abs()));
  EXPECT_EQ(MACRO_DARG(-Time(-971, 973).ToNSec()),
            MACRO_DARG(Time(970, Time::kNSecInSec - 973).ToNSec()));
}

TEST(TimeTest, FromRate) {
  EXPECT_EQ(::std::chrono::milliseconds(10), Time::FromRate(100));
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
