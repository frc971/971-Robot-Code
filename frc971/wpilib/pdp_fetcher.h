#ifndef FRC971_WPILIB_PDP_FETCHER_H_
#define FRC971_WPILIB_PDP_FETCHER_H_

#include <memory>
#include <atomic>

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/mutex.h"

#include "PowerDistributionPanel.h"

namespace frc971 {
namespace wpilib {

// Handles fetching values from the PDP. This is slow, so it has to happen in a
// separate thread.
class PDPFetcher {
 public:
  PDPFetcher();

  void Quit() { run_ = false; }

  // Retrieves the latest set of values and stores it in *pdp_values.
  // This is safe to call from any thread.
  void GetValues(::aos::PDPValues *pdp_values);

  // To be called by a ::std::thread.
  void operator()();

 private:
  const ::std::unique_ptr<PowerDistributionPanel> pdp_;

  ::aos::PDPValues pdp_values_;
  ::aos::Mutex values_lock_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_PDP_FETCHER_H_
