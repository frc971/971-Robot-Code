#ifndef FRC971_WPILIB_PDP_FETCHER_H_
#define FRC971_WPILIB_PDP_FETCHER_H_

#include <atomic>
#include <memory>

#include "aos/events/event-loop.h"
#include "frc971/wpilib/pdp_values.q.h"

namespace frc971 {
namespace wpilib {

// Handles fetching values from the PDP. This is slow, so it has to happen in a
// separate thread.
class PDPFetcher {
 public:
  PDPFetcher(::aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        pdp_values_sender_(event_loop_->MakeSender<::frc971::PDPValues>(
            ".frc971.pdp_values")) {}

  void Quit() { run_ = false; }

  // To be called by a ::std::thread.
  void operator()();

 private:
  ::aos::EventLoop *event_loop_;

  ::aos::Sender<::frc971::PDPValues> pdp_values_sender_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_PDP_FETCHER_H_
