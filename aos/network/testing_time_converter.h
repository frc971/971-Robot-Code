#ifndef AOS_NETWORK_TESTING_TIME_CONVERTER_H_
#define AOS_NETWORK_TESTING_TIME_CONVERTER_H_

#include <deque>
#include <optional>
#include <tuple>

#include "aos/events/event_scheduler.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/time/time.h"

namespace aos {
namespace message_bridge {

// Simple class to which uses InterpolatedTimeConverter to produce an
// interpolated timeline.  Should only be used for testing.
class TestingTimeConverter final : public InterpolatedTimeConverter {
 public:
  TestingTimeConverter(size_t node_count);

  virtual ~TestingTimeConverter();

  // Starts all nodes equal to the distributed clock.
  void StartEqual();

  // Elapses each node's clock by the provided duration, and returns the
  // duration that the distributed clock elapsed by.
  std::chrono::nanoseconds AddMonotonic(
      std::vector<monotonic_clock::duration> times);

  // Sets time on each node's clock to the provided times, and returns the
  // duration that the distributed clock elapsed by.  Note: time must always go
  // forwards.
  std::chrono::nanoseconds AddMonotonic(
      std::vector<logger::BootTimestamp> times);

  void RebootAt(size_t node_index, distributed_clock::time_point t);

  // Adds a distributed to monotonic clock mapping to the queue.
  void AddNextTimestamp(distributed_clock::time_point time,
                        std::vector<logger::BootTimestamp> times);

  std::optional<std::tuple<distributed_clock::time_point,
                           std::vector<logger::BootTimestamp>>>
  NextTimestamp() override;

  void set_boot_uuid(size_t node_index, size_t boot_count, UUID uuid) {
    CHECK(boot_uuids_
              .emplace(std::make_pair(node_index, boot_count), std ::move(uuid))
              .second)
        << ": Duplicate boot";
  }

  UUID boot_uuid(size_t node_index, size_t boot_count) override {
    auto it = boot_uuids_.find(std::make_pair(node_index, boot_count));
    if (it != boot_uuids_.end()) return it->second;

    auto new_it = boot_uuids_.emplace(std::make_pair(node_index, boot_count),
                                      UUID::Random());
    CHECK(new_it.second);
    return new_it.first->second;
  }

 private:
  // List of timestamps.
  std::deque<std::tuple<distributed_clock::time_point,
                        std::vector<logger::BootTimestamp>>>
      ts_;

  // True if there is no time queued.
  bool first_ = true;
  // The last times returned on all clocks.
  distributed_clock::time_point last_distributed_ = distributed_clock::epoch();
  std::vector<logger::BootTimestamp> last_monotonic_;

  std::map<std::pair<size_t, size_t>, UUID> boot_uuids_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_TESTING_TIME_CONVERTER_H_
