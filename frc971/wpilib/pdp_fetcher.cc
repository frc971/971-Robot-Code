#include "frc971/wpilib/pdp_fetcher.h"

#include "aos/common/logging/queue_logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/util/phased_loop.h"

namespace frc971 {
namespace wpilib {

PDPFetcher::PDPFetcher() : pdp_(new PowerDistributionPanel()) {
  pdp_values_.Zero();
}

void PDPFetcher::GetValues(::aos::PDPValues *pdp_values) {
  ::aos::MutexLocker locker(&values_lock_);
  *pdp_values = pdp_values_;
}

void PDPFetcher::operator()() {
  ::aos::SetCurrentThreadName("PDPFetcher");

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(20),
                                      ::aos::time::Time::InMS(4));

  while (true) {
    {
      const int iterations = phased_loop.SleepUntilNext();
      if (iterations != 1) {
        LOG(DEBUG, "PDPFetcher skipped %d iterations\n", iterations - 1);
      }
    }
    {
      const double voltage = pdp_->GetVoltage();
      ::aos::MutexLocker locker(&values_lock_);
      pdp_values_.voltage = voltage;
    }
    {
      const double temperature = pdp_->GetTemperature();
      ::aos::MutexLocker locker(&values_lock_);
      pdp_values_.temperature = temperature;
    }
    {
      const double power = pdp_->GetTotalPower();
      ::aos::MutexLocker locker(&values_lock_);
      pdp_values_.power = power;
    }
    for (int i = 0; i < 16; ++i) {
      const double current = pdp_->GetCurrent(i);
      ::aos::MutexLocker locker(&values_lock_);
      pdp_values_.currents[i] = current;
    }
    {
      ::aos::MutexLocker locker(&values_lock_);
      LOG_STRUCT(DEBUG, "finished fetching", pdp_values_);
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
