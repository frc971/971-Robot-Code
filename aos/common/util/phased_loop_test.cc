#include "aos/common/util/phased_loop.h"

#include "gtest/gtest.h"

#include "aos/testing/test_logging.h"

namespace aos {
namespace time {
namespace testing {

class PhasedLoopTest : public ::testing::Test {
 protected:
  PhasedLoopTest() {
    ::aos::testing::EnableTestLogging();
  }
};

typedef PhasedLoopTest PhasedLoopDeathTest;

TEST_F(PhasedLoopTest, Reset) {
  {
    PhasedLoop loop(Time::InMS(100), Time::kZero);

    loop.Reset(Time::kZero);
    EXPECT_EQ(Time::InMS(0), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::kZero));
    EXPECT_EQ(Time::InMS(100), loop.sleep_time());

    loop.Reset(Time::InMS(99));
    EXPECT_EQ(Time::InMS(0), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(99)));
    EXPECT_EQ(Time::InMS(100), loop.sleep_time());

    loop.Reset(Time::InMS(100));
    EXPECT_EQ(Time::InMS(100), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(199)));
    EXPECT_EQ(Time::InMS(200), loop.sleep_time());

    loop.Reset(Time::InMS(101));
    EXPECT_EQ(Time::InMS(100), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(101)));
    EXPECT_EQ(Time::InMS(200), loop.sleep_time());
  }
  {
    PhasedLoop loop(Time::InMS(100), Time::InMS(1));
    loop.Reset(Time::kZero);
    EXPECT_EQ(Time::InMS(-99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::kZero));
    EXPECT_EQ(Time::InMS(1), loop.sleep_time());
  }
  {
    PhasedLoop loop(Time::InMS(100), Time::InMS(99));

    loop.Reset(Time::kZero);
    EXPECT_EQ(Time::InMS(-1), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::kZero));
    EXPECT_EQ(Time::InMS(99), loop.sleep_time());

    loop.Reset(Time::InMS(98));
    EXPECT_EQ(Time::InMS(-1), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(98)));
    EXPECT_EQ(Time::InMS(99), loop.sleep_time());

    loop.Reset(Time::InMS(99));
    EXPECT_EQ(Time::InMS(99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(99)));
    EXPECT_EQ(Time::InMS(199), loop.sleep_time());

    loop.Reset(Time::InMS(100));
    EXPECT_EQ(Time::InMS(99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(100)));
    EXPECT_EQ(Time::InMS(199), loop.sleep_time());
  }
}

TEST_F(PhasedLoopTest, Iterate) {
  {
    PhasedLoop loop(Time::InMS(100), Time::InMS(99));
    loop.Reset(Time::kZero);
    EXPECT_EQ(1, loop.Iterate(Time::kZero));
    EXPECT_EQ(Time::InMS(99), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(100)));
    EXPECT_EQ(Time::InMS(199), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(Time::InMS(100)));
    EXPECT_EQ(Time::InMS(199), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(Time::InMS(101)));
    EXPECT_EQ(Time::InMS(199), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(Time::InMS(198)));
    EXPECT_EQ(Time::InMS(199), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(199)));
    EXPECT_EQ(Time::InMS(299), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(300)));
    EXPECT_EQ(Time::InMS(399), loop.sleep_time());
    EXPECT_EQ(3, loop.Iterate(Time::InMS(600)));
    EXPECT_EQ(Time::InMS(699), loop.sleep_time());
  }
  {
    PhasedLoop loop(Time::InMS(100), Time::InMS(1));
    loop.Reset(Time::kZero);
    EXPECT_EQ(1, loop.Iterate(Time::kZero));
    EXPECT_EQ(Time::InMS(1), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(100)));
    EXPECT_EQ(Time::InMS(101), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(Time::InMS(100)));
    EXPECT_EQ(Time::InMS(101), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(103)));
    EXPECT_EQ(Time::InMS(201), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(Time::InMS(198)));
    EXPECT_EQ(Time::InMS(201), loop.sleep_time());
    EXPECT_EQ(0, loop.Iterate(Time::InMS(200)));
    EXPECT_EQ(Time::InMS(201), loop.sleep_time());
    EXPECT_EQ(1, loop.Iterate(Time::InMS(201)));
    EXPECT_EQ(Time::InMS(301), loop.sleep_time());
    EXPECT_EQ(3, loop.Iterate(Time::InMS(600)));
    EXPECT_EQ(Time::InMS(601), loop.sleep_time());
  }
}

// Makes sure that everything works correctly when crossing zero.
// This seems like a rare case at first, but starting from zero needs to
// work, which means negatives should too.
TEST_F(PhasedLoopTest, CrossingZero) {
  PhasedLoop loop(Time::InMS(100), Time::InMS(1));
  loop.Reset(Time::InMS(-1000));
  EXPECT_EQ(Time::InMS(-1099), loop.sleep_time());
  EXPECT_EQ(9, loop.Iterate(Time::InMS(-250)));
  EXPECT_EQ(Time::InMS(-199), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(Time::InMS(-199)));
  EXPECT_EQ(Time::InMS(-99), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(Time::InMS(-90)));
  EXPECT_EQ(Time::InMS(1), loop.sleep_time());
  EXPECT_EQ(0, loop.Iterate(Time::InMS(0)));
  EXPECT_EQ(Time::InMS(1), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(Time::InMS(1)));
  EXPECT_EQ(Time::InMS(101), loop.sleep_time());

  EXPECT_EQ(0, loop.Iterate(Time::InMS(2)));
  EXPECT_EQ(Time::InMS(101), loop.sleep_time());

  EXPECT_EQ(-2, loop.Iterate(Time::InMS(-101)));
  EXPECT_EQ(Time::InMS(-99), loop.sleep_time());
  EXPECT_EQ(1, loop.Iterate(Time::InMS(-99)));
  EXPECT_EQ(Time::InMS(1), loop.sleep_time());

  EXPECT_EQ(0, loop.Iterate(Time::InMS(-99)));
  EXPECT_EQ(Time::InMS(1), loop.sleep_time());
}

// Tests that passing invalid values to the constructor dies correctly.
TEST_F(PhasedLoopDeathTest, InvalidValues) {
  EXPECT_DEATH(PhasedLoop(Time::InMS(1), Time::InMS(2)), ".*offset<interval.*");
  EXPECT_DEATH(PhasedLoop(Time::InMS(1), Time::InMS(1)), ".*offset<interval.*");
  EXPECT_DEATH(PhasedLoop(Time::InMS(1), Time::InMS(-1)),
               ".*offset>=Time::kZero.*");
  EXPECT_DEATH(PhasedLoop(Time::InMS(0), Time::InMS(0)),
               ".*interval>Time::kZero.*");
}

}  // namespace testing
}  // namespace time
}  // namespace aos
