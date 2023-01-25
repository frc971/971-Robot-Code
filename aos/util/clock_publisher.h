#ifndef AOS_UTIL_CLOCK_PUBLISHER_H_
#define AOS_UTIL_CLOCK_PUBLISHER_H_
#include "aos/events/simulated_event_loop.h"
#include "aos/util/clock_timepoints_generated.h"

namespace aos {
// A simple class that periodically queries a SimulatedEventLoopFactory for the
// current timestamps on all nodes and publishes a ClockTimepoints message on
// the provided EventLoop.
// This is used by the log_to_mcap converter to allow Foxglove users access to
// offset estimates. In order to use this, a /clocks channel with a type of
// aos.ClockTimepoints must be available.
class ClockPublisher {
 public:
  ClockPublisher(aos::SimulatedEventLoopFactory *factory,
                 aos::EventLoop *event_loop);

 private:
  void SendTimepoints();

  aos::SimulatedEventLoopFactory *const factory_;
  aos::Sender<ClockTimepoints> timepoints_sender_;
};
}  // namespace aos

#endif  // AOS_UTIL_CLOCK_PUBLISHER_H_
