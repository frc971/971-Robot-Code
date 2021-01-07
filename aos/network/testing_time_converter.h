#ifndef AOS_NETWORK_TESTING_TIME_CONVERTER_H_
#define AOS_NETWORK_TESTING_TIME_CONVERTER_H_

#include <deque>
#include <optional>
#include <tuple>

#include "aos/events/event_scheduler.h"
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
      std::vector<monotonic_clock::time_point> times);

  // Adds a distributed to monotonic clock mapping to the queue.
  void AddNextTimestamp(distributed_clock::time_point time,
                        std::vector<monotonic_clock::time_point> times);

  std::optional<std::tuple<distributed_clock::time_point,
                           std::vector<monotonic_clock::time_point>>>
  NextTimestamp() override;

 private:
  // List of timestamps.
  std::deque<std::tuple<distributed_clock::time_point,
                        std::vector<monotonic_clock::time_point>>>
      ts_;

  // True if there is no time queued.
  bool first_ = true;
  // The last times returned on all clocks.
  distributed_clock::time_point last_distributed_ = distributed_clock::epoch();
  std::vector<monotonic_clock::time_point> last_monotonic_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_TESTING_TIME_CONVERTER_H_
