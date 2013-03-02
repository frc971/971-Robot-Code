#include "aos/common/time.h"

#include "gtest/gtest.h"

#include "aos/common/macros.h"

namespace aos {
namespace time {
namespace testing {

TEST(TimeTest, timespecConversions) {
  timespec start{1234, 5678};  // NOLINT
  Time time(start);
  EXPECT_EQ(start.tv_sec, static_cast<signed time_t>(time.sec()));
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

// It's kind of hard not to test Now and SleepFor at the same time.
TEST(TimeTest, NowAndSleepFor) {
  // without this, it tends to fail the first time (ends up sleeping for way
  // longer than it should the second time, where it actually matters)
  SleepFor(Time(0, Time::kNSecInSec / 10));
  Time start = Time::Now();
  SleepFor(Time(0, Time::kNSecInSec * 2 / 10));
  EXPECT_TRUE(MACRO_DARG((Time::Now() - start)
                         .IsWithin(Time(0, Time::kNSecInSec * 2 / 10),
                                   Time::kNSecInSec / 1000)));
}

TEST(TimeTest, AbsoluteSleep) {
  Time start = Time::Now();
  SleepFor(Time(0, Time::kNSecInSec / 10));
  SleepUntil((start + Time(0, Time::kNSecInSec * 2 / 10)));
  EXPECT_TRUE(MACRO_DARG((Time::Now() - start)
                         .IsWithin(Time(0, Time::kNSecInSec * 2 / 10),
                                   Time::kNSecInSec / 1000)));
}

TEST(TimeTest, Addition) {
  Time t(54, 500);
  EXPECT_EQ(MACRO_DARG(Time(54, 5500)), t + MACRO_DARG(Time(0, 5000)));
  EXPECT_EQ(MACRO_DARG(Time(56, 500)), t + MACRO_DARG(Time(2, 0)));
  EXPECT_EQ(MACRO_DARG(Time(57, 6500)), t + MACRO_DARG(Time(3, 6000)));
  EXPECT_EQ(MACRO_DARG(Time(50, 300)),
            t + MACRO_DARG(Time(-5, Time::kNSecInSec - 200)));
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
}

TEST(TimeTest, Multiplication) {
  Time t(54, Time::kNSecInSec / 3);
  EXPECT_EQ(MACRO_DARG(Time(108, Time::kNSecInSec / 3 * 2)), t * 2);
  EXPECT_EQ(MACRO_DARG(Time(271, Time::kNSecInSec / 3 * 2 - 1)), t * 5);
}
TEST(TimeTest, Division) {
  EXPECT_EQ(MACRO_DARG(Time(5, Time::kNSecInSec / 10 * 4 + 50)),
            MACRO_DARG(Time(54, 500)) / 10);
}

TEST(TimeTest, Comparisons) {
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) > Time(971, 253)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) >= Time(971, 253)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) < Time(971, 255)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) <= Time(971, 255)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) >= Time(971, 253)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) <= Time(971, 254)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) >= Time(971, 254)));
  EXPECT_TRUE(MACRO_DARG(Time(972, 254) > Time(971, 254)));
  EXPECT_TRUE(MACRO_DARG(Time(971, 254) < Time(972, 254)));
}

TEST(TimeTest, Within) {
  EXPECT_TRUE(MACRO_DARG(Time(55, 5000).IsWithin(Time(55, 4900), 100)));
  EXPECT_FALSE(MACRO_DARG(Time(55, 5000).IsWithin(Time(55, 4900), 99)));
  EXPECT_TRUE(MACRO_DARG(Time(5, 0).IsWithin(Time(4, Time::kNSecInSec - 200),
                                             250)));
}

TEST(TimeTest, Modulo) {
  EXPECT_EQ(MACRO_DARG(Time(0, Time::kNSecInSec / 10 * 2)),
            MACRO_DARG(Time(50, 0) % (Time::kNSecInSec / 10 * 3)));
}

TEST(TimeTest, InSeconds) {
  EXPECT_EQ(MACRO_DARG(Time(2, Time::kNSecInSec / 100 * 55 - 1)),
            Time::InSeconds(2.55));
}

TEST(TimeTest, ToSeconds) {
  EXPECT_EQ(13.23, Time::InSeconds(13.23).ToSeconds());
}

#ifdef __VXWORKS__
TEST(TimeTest, ToTicks) {
  EXPECT_EQ(sysClkRateGet() / 100,
            MACRO_DARG(Time(0, Time::kNSecInSec / 100).ToTicks()));
}
#endif

TEST(TimeTest, InMS) {
  Time t = Time::InMS(254971);
  EXPECT_EQ(254, t.sec());
  EXPECT_EQ(971000000, t.nsec());
}

TEST(TimeTest, InNS) {
  Time t = Time::InNS(static_cast<int64_t>(973254111971ll));
  EXPECT_EQ(973, t.sec());
  EXPECT_EQ(254111971, t.nsec());
}

TEST(TimeTest, InUS) {
  Time t = Time::InUS(254111971);
  EXPECT_EQ(254, t.sec());
  EXPECT_EQ(111971000, t.nsec());
}

TEST(TimeTest, ToMSec) {
  Time t(254, 971000000);
  EXPECT_EQ(254971, t.ToMSec());
}

}  // namespace testing
}  // namespace time
}  // namespace aos
