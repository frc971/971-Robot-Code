#ifndef FRC971_WPILIB_PDP_FETCHER_H_
#define FRC971_WPILIB_PDP_FETCHER_H_

#include <atomic>
#include <memory>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/wpilib/pdp_values_generated.h"

namespace frc {
class PowerDistributionPanel;
}  // namespace frc

namespace frc971 {
namespace wpilib {

// Handles fetching values from the PDP.
class PDPFetcher {
 public:
  PDPFetcher(::aos::ShmEventLoop *event_loop);

  ~PDPFetcher();

 private:
  void Loop(int iterations);

  ::aos::EventLoop *event_loop_;

  ::aos::Sender<::frc971::PDPValues> pdp_values_sender_;

  ::std::unique_ptr<::frc::PowerDistributionPanel> pdp_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_PDP_FETCHER_H_
