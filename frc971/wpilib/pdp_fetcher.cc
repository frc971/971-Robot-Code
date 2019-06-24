#include "frc971/wpilib/pdp_fetcher.h"

#include <chrono>

#include "aos/events/event-loop.h"
#include "aos/init.h"
#include "aos/logging/queue_logging.h"
#include "aos/util/phased_loop.h"
#include "frc971/wpilib/ahal/PowerDistributionPanel.h"
#include "frc971/wpilib/pdp_values.q.h"

namespace frc971 {
namespace wpilib {

void PDPFetcher::operator()() {
  ::aos::SetCurrentThreadName("PDPFetcher");
  ::std::unique_ptr<frc::PowerDistributionPanel> pdp(
      new frc::PowerDistributionPanel());

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(20),
                                      ::aos::monotonic_clock::now(),
                                      ::std::chrono::milliseconds(4));

  // TODO(austin): Event loop instead of while loop.
  while (true) {
    {
      const int iterations = phased_loop.SleepUntilNext();
      if (iterations != 1) {
        LOG(DEBUG, "PDPFetcher skipped %d iterations\n", iterations - 1);
      }
    }
    auto message = pdp_values_sender_.MakeMessage();
    message->voltage = pdp->GetVoltage();
    message->temperature = pdp->GetTemperature();
    message->power = pdp->GetTotalPower();
    for (int i = 0; i < 16; ++i) {
      message->currents[i] = pdp->GetCurrent(i);
    }
    LOG_STRUCT(DEBUG, "got", *message);
    if (!message.Send()) {
      LOG(WARNING, "sending pdp values failed\n");
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
