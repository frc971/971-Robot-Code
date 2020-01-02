#ifndef AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
#define AOS_EVENTS_SIMULATED_EVENT_LOOP_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_scheduler.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/ipc_lib/index.h"
#include "glog/logging.h"

namespace aos {

// Class for simulated fetchers.
class SimulatedChannel;

struct SimpleChannel {
  SimpleChannel(const Channel *channel);
  std::string name;
  std::string type;

  std::string DebugString() const {
    return std::string("{ ") + name + ", " + type + "}";
  }

  bool operator==(const SimpleChannel &other) const {
    return name == other.name && type == other.type;
  }
  bool operator<(const SimpleChannel &other) const {
    int name_compare = other.name.compare(name);
    if (name_compare == 0) {
      return other.type < type;
    } else if (name_compare < 0) {
      return true;
    } else {
      return false;
    }
  }
};

class SimulatedEventLoopFactory {
 public:
  // Constructs a SimulatedEventLoopFactory with the provided configuration.
  // This configuration must remain in scope for the lifetime of the factory and
  // all sub-objects.
  SimulatedEventLoopFactory(const Configuration *configuration);
  SimulatedEventLoopFactory(const Configuration *configuration,
                            std::string_view node_name);
  SimulatedEventLoopFactory(const Configuration *configuration,
                            const Node *node);
  ~SimulatedEventLoopFactory();

  ::std::unique_ptr<EventLoop> MakeEventLoop(std::string_view name);

  // Starts executing the event loops unconditionally.
  void Run();
  // Executes the event loops for a duration.
  void RunFor(monotonic_clock::duration duration);

  // Stops executing all event loops.  Meant to be called from within an event
  // loop handler.
  void Exit() { scheduler_.Exit(); }

  // Sets the simulated send delay for the factory.
  void set_send_delay(std::chrono::nanoseconds send_delay);

  // Returns the node that this factory is running as, or nullptr if this is a
  // single node setup.
  const Node *node() const { return node_; }

  monotonic_clock::time_point monotonic_now() const {
    return scheduler_.monotonic_now();
  }
  realtime_clock::time_point realtime_now() const {
    return scheduler_.realtime_now();
  }

  // Sets realtime clock to realtime_now for a given monotonic clock.
  void SetRealtimeOffset(monotonic_clock::time_point monotonic_now,
                         realtime_clock::time_point realtime_now) {
    scheduler_.SetRealtimeOffset(monotonic_now, realtime_now);
  }

 private:
  const Configuration *const configuration_;
  EventScheduler scheduler_;
  // Map from name, type to queue.
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> channels_;
  // List of event loops to manage running and not running for.
  std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
      raw_event_loops_;

  std::chrono::nanoseconds send_delay_ = std::chrono::microseconds(50);

  const Node *const node_;

  pid_t tid_ = 0;
};

}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
