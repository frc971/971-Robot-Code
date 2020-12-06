#ifndef AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
#define AOS_EVENTS_SIMULATED_EVENT_LOOP_H_

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_scheduler.h"
#include "aos/events/logging/uuid.h"
#include "aos/events/simple_channel.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/ipc_lib/index.h"
#include "glog/logging.h"

namespace aos {

// Class for simulated fetchers.
class SimulatedChannel;

class NodeEventLoopFactory;
namespace message_bridge {
class SimulatedMessageBridge;
}

// There are 2 concepts needed to support multi-node simulations.
//  1) The node.  This is implemented with NodeEventLoopFactory.
//  2) The "robot" which runs multiple nodes.  This is implemented with
//     SimulatedEventLoopFactory.
//
// To make things easier, SimulatedEventLoopFactory takes an optional Node
// argument if you want to make event loops without interacting with the
// NodeEventLoopFactory object.
//
// The basic flow goes something like as follows:
//
// SimulatedEventLoopFactory factory(config);
// const Node *pi1 = configuration::GetNode(factory.configuration(), "pi1");
// std::unique_ptr<EventLoop> event_loop = factory.MakeEventLoop("ping", pi1);
//
// Or
//
// SimulatedEventLoopFactory factory(config);
// const Node *pi1 = configuration::GetNode(factory.configuration(), "pi1");
// NodeEventLoopFactory *pi1_factory = factory.GetNodeEventLoopFactory(pi1);
// std::unique_ptr<EventLoop> event_loop = pi1_factory.MakeEventLoop("ping");
//
// The distributed_clock is used to be the base time.  NodeEventLoopFactory has
// all the information needed to adjust both the realtime and monotonic clocks
// relative to the distributed_clock.
class SimulatedEventLoopFactory {
 public:
  // Constructs a SimulatedEventLoopFactory with the provided configuration.
  // This configuration must remain in scope for the lifetime of the factory and
  // all sub-objects.
  SimulatedEventLoopFactory(const Configuration *configuration);
  ~SimulatedEventLoopFactory();

  // Creates an event loop.  If running in a multi-node environment, node needs
  // to point to the node to create this event loop on.
  ::std::unique_ptr<EventLoop> MakeEventLoop(std::string_view name,
                                             const Node *node = nullptr);

  // Returns the NodeEventLoopFactory for the provided node.  The returned
  // NodeEventLoopFactory is owned by the SimulatedEventLoopFactory and has a
  // lifetime identical to the factory.
  NodeEventLoopFactory *GetNodeEventLoopFactory(const Node *node);

  // Starts executing the event loops unconditionally.
  void Run();
  // Executes the event loops for a duration.
  void RunFor(distributed_clock::duration duration);

  // Stops executing all event loops.  Meant to be called from within an event
  // loop handler.
  void Exit();

  const std::vector<const Node *> &nodes() const { return nodes_; }

  // Sets the simulated send delay for all messages sent within a single node.
  void set_send_delay(std::chrono::nanoseconds send_delay);
  std::chrono::nanoseconds send_delay() const { return send_delay_; }

  // Sets the simulated network delay for messages forwarded between nodes.
  void set_network_delay(std::chrono::nanoseconds network_delay) {
    network_delay_ = network_delay;
  }
  std::chrono::nanoseconds network_delay() const { return network_delay_; }

  // Returns the clock used to synchronize the nodes.
  distributed_clock::time_point distributed_now() const {
    return scheduler_scheduler_.distributed_now();
  }

  // Returns the configuration used for everything.
  const Configuration *configuration() const { return configuration_; }

  // Disables forwarding for this channel.  This should be used very rarely only
  // for things like the logger.
  void DisableForwarding(const Channel *channel);

  // Disables the messages sent by the simulated message gateway.
  void DisableStatistics();

 private:
  const Configuration *const configuration_;
  EventSchedulerScheduler scheduler_scheduler_;
  // List of event loops to manage running and not running for.
  // The function is a callback used to set and clear the running bool on each
  // event loop.
  std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
      raw_event_loops_;

  std::chrono::nanoseconds send_delay_ = std::chrono::microseconds(50);
  std::chrono::nanoseconds network_delay_ = std::chrono::microseconds(100);

  std::vector<std::unique_ptr<NodeEventLoopFactory>> node_factories_;

  std::vector<const Node *> nodes_;

  std::unique_ptr<message_bridge::SimulatedMessageBridge> bridge_;
};

// This class holds all the state required to be a single node.
class NodeEventLoopFactory {
 public:
  ::std::unique_ptr<EventLoop> MakeEventLoop(std::string_view name);

  // Returns the node that this factory is running as, or nullptr if this is a
  // single node setup.
  const Node *node() const { return node_; }

  // Sets realtime clock to realtime_now for a given monotonic clock.
  void SetRealtimeOffset(monotonic_clock::time_point monotonic_now,
                         realtime_clock::time_point realtime_now) {
    realtime_offset_ =
        realtime_now.time_since_epoch() - monotonic_now.time_since_epoch();
  }

  // Returns the current time on both clocks.
  inline monotonic_clock::time_point monotonic_now() const;
  inline realtime_clock::time_point realtime_now() const;

  const Configuration *configuration() const {
    return factory_->configuration();
  }

  // Returns the simulated network delay for messages forwarded between nodes.
  std::chrono::nanoseconds network_delay() const {
    return factory_->network_delay();
  }
  // Returns the simulated send delay for all messages sent within a single
  // node.
  std::chrono::nanoseconds send_delay() const { return factory_->send_delay(); }

  // TODO(austin): Private for the following?

  // Converts a time to the distributed clock for scheduling and cross-node time
  // measurement.
  inline distributed_clock::time_point ToDistributedClock(
      monotonic_clock::time_point time) const;
  inline monotonic_clock::time_point FromDistributedClock(
      distributed_clock::time_point time) const;

  // Sets the offset between the monotonic clock and the central distributed
  // clock.  distributed_clock = monotonic_clock + offset.
  void SetDistributedOffset(std::chrono::nanoseconds monotonic_offset,
                            double monotonic_slope) {
    scheduler_.SetDistributedOffset(monotonic_offset, monotonic_slope);
  }

  // Returns the boot UUID for this node.
  const UUID &boot_uuid() const { return boot_uuid_; }

  // Reboots the node.  This just resets the boot_uuid_, nothing else.
  // TODO(austin): This is here for a test case or two, not for general
  // consumption.  The interactions with the rest of the system need to be
  // worked out better.  Don't use this for anything real yet.
  void Reboot() { boot_uuid_ = UUID::Random(); }

 private:
  friend class SimulatedEventLoopFactory;
  NodeEventLoopFactory(
      EventSchedulerScheduler *scheduler_scheduler,
      SimulatedEventLoopFactory *factory, const Node *node,
      std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
          *raw_event_loops);

  EventScheduler scheduler_;
  SimulatedEventLoopFactory *const factory_;

  UUID boot_uuid_ = UUID::Random();

  const Node *const node_;

  std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
      *const raw_event_loops_;

  std::chrono::nanoseconds realtime_offset_ = std::chrono::seconds(0);

  // Map from name, type to queue.
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> channels_;

  // pid so we get unique timing reports.
  pid_t tid_ = 0;
};

inline monotonic_clock::time_point NodeEventLoopFactory::monotonic_now() const {
  // TODO(austin): Confirm that time never goes backwards?
  return scheduler_.monotonic_now();
}

inline realtime_clock::time_point NodeEventLoopFactory::realtime_now() const {
  return realtime_clock::time_point(monotonic_now().time_since_epoch() +
                                    realtime_offset_);
}

inline monotonic_clock::time_point NodeEventLoopFactory::FromDistributedClock(
    distributed_clock::time_point time) const {
  return scheduler_.FromDistributedClock(time);
}

inline distributed_clock::time_point NodeEventLoopFactory::ToDistributedClock(
    monotonic_clock::time_point time) const {
  return scheduler_.ToDistributedClock(time);
}

}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
