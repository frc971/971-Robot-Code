#include "frc971/wpilib/pdp_fetcher.h"

#include "aos/common/logging/queue_logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/util/phased_loop.h"
#include "frc971/wpilib/pdp_values.q.h"

namespace frc971 {
namespace wpilib {

void PDPFetcher::operator()() {
  ::aos::SetCurrentThreadName("PDPFetcher");
  ::std::unique_ptr<PowerDistributionPanel> pdp(new PowerDistributionPanel());

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(20),
                                      ::aos::time::Time::InMS(4));

  while (true) {
    {
      const int iterations = phased_loop.SleepUntilNext();
      if (iterations != 1) {
        LOG(DEBUG, "PDPFetcher skipped %d iterations\n", iterations - 1);
      }
    }
    auto message = pdp_values.MakeMessage();
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
