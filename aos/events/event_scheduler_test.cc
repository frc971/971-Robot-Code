#include "aos/events/event_scheduler.h"

#include <chrono>

#include "gtest/gtest.h"

namespace aos {

namespace chrono = std::chrono;

// Tests that the default parameters (slope of 1, offest of 0) behave as
// an identity.
TEST(EventSchedulerTest, IdentityTimeConversion) {
  EventScheduler s;
  EXPECT_EQ(s.FromDistributedClock(distributed_clock::epoch()),
            monotonic_clock::epoch());

  EXPECT_EQ(
      s.FromDistributedClock(distributed_clock::epoch() + chrono::seconds(1)),
      monotonic_clock::epoch() + chrono::seconds(1));

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch()),
            distributed_clock::epoch());

  EXPECT_EQ(
      s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(1)),
      distributed_clock::epoch() + chrono::seconds(1));
}

// Tests that a non-unity slope is computed correctly.
TEST(EventSchedulerTest, DoubleTimeConversion) {
  EventScheduler s;
  s.SetDistributedOffset(std::chrono::seconds(7), 2.0);

  EXPECT_EQ(s.FromDistributedClock(distributed_clock::epoch()),
            monotonic_clock::epoch() + chrono::seconds(7));

  EXPECT_EQ(
      s.FromDistributedClock(distributed_clock::epoch() + chrono::seconds(1)),
      monotonic_clock::epoch() + chrono::seconds(9));

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(7)),
            distributed_clock::epoch());

  EXPECT_EQ(
      s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(9)),
      distributed_clock::epoch() + chrono::seconds(1));
}

}  // namespace aos
