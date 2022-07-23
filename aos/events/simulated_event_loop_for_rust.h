#ifndef AOS_EVENTS_SIMULATED_EVENT_LOOP_FOR_RUST_H_
#define AOS_EVENTS_SIMULATED_EVENT_LOOP_FOR_RUST_H_

#include "aos/events/simulated_event_loop.h"
#include "aos/for_rust.h"
#include "cxx.h"

namespace aos {

class SimulatedEventLoopFactoryForRust {
 public:
  SimulatedEventLoopFactoryForRust(const Configuration *configuration)
      : factory_(configuration) {}

  std::unique_ptr<EventLoop> MakeEventLoop(rust::Str name, const Node *node) {
    return factory_.MakeEventLoop(RustStrToStringView(name), node);
  }

  void Run() { factory_.Run(); }

  std::unique_ptr<ExitHandle> MakeExitHandle() {
    return factory_.MakeExitHandle();
  }

  const Configuration *configuration() { return factory_.configuration(); }

 private:
  SimulatedEventLoopFactory factory_;
};

}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_EVENT_LOOP_FOR_RUST_H_
