#include "aos/events/event_scheduler.h"

#include <chrono>

#include "gtest/gtest.h"

namespace aos {

namespace chrono = std::chrono;
using aos::logger::BootTimestamp;

// Legacy time converter for keeping old tests working.  Has numerical precision
// problems.
class SlopeOffsetTimeConverter final : public TimeConverter {
 public:
  SlopeOffsetTimeConverter(size_t nodes_count)
      : distributed_offset_(nodes_count, std::chrono::seconds(0)),
        distributed_slope_(nodes_count, 1.0) {
    uuids_.reserve(nodes_count);
    while (uuids_.size() < nodes_count) {
      uuids_.emplace_back(UUID::Random());
    }
  }

  // Sets the offset between the distributed and monotonic clock.
  //   monotonic = distributed * slope + offset;
  void SetDistributedOffset(size_t node_index,
                            std::chrono::nanoseconds distributed_offset,
                            double distributed_slope) {
    distributed_offset_[node_index] = distributed_offset;
    distributed_slope_[node_index] = distributed_slope;
  }

  distributed_clock::time_point ToDistributedClock(
      size_t node_index, BootTimestamp time) override {
    CHECK_EQ(time.boot, 0u);
    return distributed_clock::epoch() +
           std::chrono::duration_cast<std::chrono::nanoseconds>(
               (time.time_since_epoch() - distributed_offset_[node_index]) /
               distributed_slope_[node_index]);
  }

  BootTimestamp FromDistributedClock(size_t node_index,
                                     distributed_clock::time_point time,
                                     size_t boot_index) override {
    CHECK_EQ(boot_index, 0u);
    return {
        .boot = 0u,
        .time = monotonic_clock::epoch() +
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    time.time_since_epoch() * distributed_slope_[node_index]) +
                distributed_offset_[node_index]};
  }

  UUID boot_uuid(size_t node_index, size_t boot_count) override {
    CHECK_EQ(boot_count, 0u);
    return uuids_[node_index];
  }

  void ObserveTimePassed(distributed_clock::time_point /*time*/) override {}

 private:
  // Offset to the distributed clock.
  //   distributed = monotonic + offset;
  std::vector<std::chrono::nanoseconds> distributed_offset_;
  std::vector<double> distributed_slope_;
  std::vector<UUID> uuids_;
};

// Tests that the default parameters (slope of 1, offest of 0) behave as
// an identity.
TEST(EventSchedulerTest, IdentityTimeConversion) {
  SlopeOffsetTimeConverter time(1);
  EventScheduler s(0);
  s.SetTimeConverter(0u, &time);
  EXPECT_EQ(s.FromDistributedClock(distributed_clock::epoch()),
            BootTimestamp::epoch());

  EXPECT_EQ(
      s.FromDistributedClock(distributed_clock::epoch() + chrono::seconds(1)),
      BootTimestamp::epoch() + chrono::seconds(1));

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch()),
            distributed_clock::epoch());

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(1)),
            distributed_clock::epoch() + chrono::seconds(1));
}

// Tests that a non-unity slope is computed correctly.
TEST(EventSchedulerTest, DoubleTimeConversion) {
  SlopeOffsetTimeConverter time(1);
  EventScheduler s(0);
  s.SetTimeConverter(0u, &time);
  time.SetDistributedOffset(0u, std::chrono::seconds(7), 2.0);

  EXPECT_EQ(s.FromDistributedClock(distributed_clock::epoch()),
            BootTimestamp::epoch() + chrono::seconds(7));

  EXPECT_EQ(
      s.FromDistributedClock(distributed_clock::epoch() + chrono::seconds(1)),
      BootTimestamp::epoch() + chrono::seconds(9));

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(7)),
            distributed_clock::epoch());

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(9)),
            distributed_clock::epoch() + chrono::seconds(1));
}

}  // namespace aos
