#ifndef FRC971_WPILIB_PDP_FETCHER_H_
#define FRC971_WPILIB_PDP_FETCHER_H_

#include <memory>
#include <atomic>

#include "PowerDistributionPanel.h"

namespace frc971 {
namespace wpilib {

// Handles fetching values from the PDP. This is slow, so it has to happen in a
// separate thread.
class PDPFetcher {
 public:
  void Quit() { run_ = false; }

  // To be called by a ::std::thread.
  void operator()();

 private:
  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_PDP_FETCHER_H_
