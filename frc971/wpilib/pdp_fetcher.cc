#include "frc971/wpilib/pdp_fetcher.h"

#include "aos/common/logging/queue_logging.h"
#include "aos/linux_code/init.h"

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
  // Something in WPILib blocks for long periods of time in here, so it's not
  // actually a busy loop like it looks. It seems to somehow be related to
  // joystick packets.
  while (true) {
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
