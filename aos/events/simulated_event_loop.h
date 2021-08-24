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
#include "aos/events/simple_channel.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/ipc_lib/index.h"
#include "aos/uuid.h"
#include "glog/logging.h"

namespace aos {

// Class for simulated fetchers.
class SimulatedChannel;

class NodeEventLoopFactory;
class SimulatedEventLoop;
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

  SimulatedEventLoopFactory(const SimulatedEventLoopFactory &) = delete;
  SimulatedEventLoopFactory &operator=(const SimulatedEventLoopFactory &) =
      delete;
  SimulatedEventLoopFactory(SimulatedEventLoopFactory &&) = delete;
  SimulatedEventLoopFactory &operator=(SimulatedEventLoopFactory &&) = delete;

  // Creates an event loop.  If running in a multi-node environment, node needs
  // to point to the node to create this event loop on.
  ::std::unique_ptr<EventLoop> MakeEventLoop(std::string_view name,
                                             const Node *node = nullptr);

  // Returns the NodeEventLoopFactory for the provided node.  The returned
  // NodeEventLoopFactory is owned by the SimulatedEventLoopFactory and has a
  // lifetime identical to the factory.
  NodeEventLoopFactory *GetNodeEventLoopFactory(const Node *node);
  NodeEventLoopFactory *GetNodeEventLoopFactory(std::string_view node);

  // Sets the time converter for all nodes.
  void SetTimeConverter(TimeConverter *time_converter);

  // Starts executing the event loops unconditionally until Exit is called or
  // all the nodes have shut down.
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

  // Calls SkipTimingReport() on all EventLoops used as part of the
  // infrastructure. This may improve the performance of long-simulated-duration
  // tests.
  void SkipTimingReport();

 private:
  friend class NodeEventLoopFactory;

  const Configuration *const configuration_;
  EventSchedulerScheduler scheduler_scheduler_;

  std::chrono::nanoseconds send_delay_ = std::chrono::microseconds(50);
  std::chrono::nanoseconds network_delay_ = std::chrono::microseconds(100);

  std::unique_ptr<message_bridge::SimulatedMessageBridge> bridge_;

  std::vector<std::unique_ptr<NodeEventLoopFactory>> node_factories_;

  std::vector<const Node *> nodes_;
};

// This class holds all the state required to be a single node.
class NodeEventLoopFactory {
 public:
  ~NodeEventLoopFactory();

  std::unique_ptr<EventLoop> MakeEventLoop(std::string_view name);

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
  inline distributed_clock::time_point distributed_now() const;

  const Configuration *configuration() const {
    return factory_->configuration();
  }

  // Starts the node up by calling the OnStartup handlers.  These get called
  // every time a node is started.

  // Called when a node has started.  This is typically when a log file starts
  // for a node.
  void OnStartup(std::function<void()> &&fn);

  // Called when a node shuts down.  These get called every time a node is shut
  // down.  All applications are destroyed right after the last OnShutdown
  // callback is called.
  void OnShutdown(std::function<void()> &&fn);

  // Starts an application if the configuration says it should be started on
  // this node.  name is the name of the application.  args are the constructor
  // args for the Main class.  Returns a pointer to the class that was started
  // if it was started, or nullptr.
  template <class Main, class... Args>
  Main *MaybeStart(std::string_view name, Args &&... args);

  // Starts an application regardless of if the config says to or not.  name is
  // the name of the application, and args are the constructor args for the
  // application.  Returns a pointer to the class that was started.
  template <class Main, class... Args>
  Main *AlwaysStart(std::string_view name, Args &&... args);

  // Returns the simulated network delay for messages forwarded between nodes.
  std::chrono::nanoseconds network_delay() const {
    return factory_->network_delay();
  }
  // Returns the simulated send delay for all messages sent within a single
  // node.
  std::chrono::nanoseconds send_delay() const { return factory_->send_delay(); }

  size_t boot_count() const { return scheduler_.boot_count(); }

  // TODO(austin): Private for the following?

  // Converts a time to the distributed clock for scheduling and cross-node time
  // measurement.
  // Note: converting time too far in the future can cause problems when
  // replaying logs.  Only convert times in the present or near past.
  inline distributed_clock::time_point ToDistributedClock(
      monotonic_clock::time_point time) const;
  inline logger::BootTimestamp FromDistributedClock(
      distributed_clock::time_point time) const;

  // Sets the class used to convert time.  This pointer must out-live the
  // SimulatedEventLoopFactory.
  void SetTimeConverter(TimeConverter *time_converter) {
    scheduler_.SetTimeConverter(
        configuration::GetNodeIndex(factory_->configuration(), node_),
        time_converter);
  }

  // Returns the boot UUID for this node.
  const UUID &boot_uuid() {
    if (boot_uuid_ == UUID::Zero()) {
      boot_uuid_ = scheduler_.boot_uuid();
    }
    return boot_uuid_;
  }

  // Stops forwarding messages to the other node, and reports disconnected in
  // the ServerStatistics message for this node, and the ClientStatistics for
  // the other node.
  void Disconnect(const Node *other);
  // Resumes forwarding messages.
  void Connect(const Node *other);

 private:
  friend class SimulatedEventLoopFactory;
  NodeEventLoopFactory(EventSchedulerScheduler *scheduler_scheduler,
                       SimulatedEventLoopFactory *factory, const Node *node);

  // Helpers to restart.
  void ScheduleStartup();
  void Startup();
  void Shutdown();

  EventScheduler scheduler_;
  SimulatedEventLoopFactory *const factory_;

  UUID boot_uuid_ = UUID::Zero();

  const Node *const node_;

  std::vector<SimulatedEventLoop *> event_loops_;

  std::chrono::nanoseconds realtime_offset_ = std::chrono::seconds(0);

  // Map from name, type to queue.
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> channels_;

  // pid so we get unique timing reports.
  pid_t tid_ = 0;

  // True if we are started.
  bool started_ = false;

  std::vector<std::function<void()>> pending_on_startup_;
  std::vector<std::function<void()>> on_startup_;
  std::vector<std::function<void()>> on_shutdown_;

  // Base class for an application to start.  This shouldn't be used directly.
  struct Application {
    Application(NodeEventLoopFactory *node_factory, std::string_view name)
        : event_loop(node_factory->MakeEventLoop(name)) {}
    virtual ~Application() {}

    std::unique_ptr<EventLoop> event_loop;
  };

  // Subclass to do type erasure for the base class.  Holds an instance of a
  // specific class.  Use SimulationStarter instead.
  template <typename Main>
  struct TypedApplication : public Application {
    // Constructs an Application by delegating the arguments used to construct
    // the event loop to Application and the rest of the args to the actual
    // application.
    template <class... Args>
    TypedApplication(NodeEventLoopFactory *node_factory, std::string_view name,
                     Args &&... args)
        : Application(node_factory, name),
          main(event_loop.get(), std::forward<Args>(args)...) {
      VLOG(1) << node_factory->scheduler_.distributed_now() << " "
              << (node_factory->node() == nullptr
                      ? ""
                      : node_factory->node()->name()->str() + " ")
              << node_factory->monotonic_now() << " Starting Application \""
              << name << "\"";
    }
    ~TypedApplication() override {}

    Main main;
  };

  std::vector<std::unique_ptr<Application>> applications_;
};

template <class Main, class... Args>
Main *NodeEventLoopFactory::MaybeStart(std::string_view name, Args &&... args) {
  const aos::Application *application =
      configuration::GetApplication(configuration(), node(), name);

  if (application != nullptr) {
    return AlwaysStart<Main>(name, std::forward<Args>(args)...);
  }
  return nullptr;
}

template <class Main, class... Args>
Main *NodeEventLoopFactory::AlwaysStart(std::string_view name,
                                        Args &&... args) {
  std::unique_ptr<TypedApplication<Main>> app =
      std::make_unique<TypedApplication<Main>>(this, name,
                                               std::forward<Args>(args)...);
  Main *main_ptr = &app->main;
  applications_.emplace_back(std::move(app));
  return main_ptr;
}

inline monotonic_clock::time_point NodeEventLoopFactory::monotonic_now() const {
  // TODO(austin): Confirm that time never goes backwards?
  return scheduler_.monotonic_now();
}

inline realtime_clock::time_point NodeEventLoopFactory::realtime_now() const {
  return realtime_clock::time_point(monotonic_now().time_since_epoch() +
                                    realtime_offset_);
}

inline distributed_clock::time_point NodeEventLoopFactory::distributed_now()
    const {
  return scheduler_.distributed_now();
}

inline logger::BootTimestamp NodeEventLoopFactory::FromDistributedClock(
    distributed_clock::time_point time) const {
  return scheduler_.FromDistributedClock(time);
}

inline distributed_clock::time_point NodeEventLoopFactory::ToDistributedClock(
    monotonic_clock::time_point time) const {
  return scheduler_.ToDistributedClock(time);
}

}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
