#include "frc971/wpilib/encoder_and_potentiometer.h"

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"

namespace frc971 {
namespace wpilib {

void DMAEncoderAndPotentiometer::UpdateFromSample(const DMASample &sample) {
  if (index_last_value_) {
    // It was already true last time, so check if it's reset back to false yet.
    index_last_value_ = sample.Get(index_.get());
  } else if (sample.Get(index_.get())) {
    // This sample is posedge, so record all the values.
    index_last_value_ = true;
    ++index_posedge_count_;
    last_encoder_value_ = sample.GetRaw(encoder_.get());
    last_potentiometer_voltage_ = sample.GetVoltage(potentiometer_.get());
  }
}

void InterruptEncoderAndPotentiometer::Start() {
  CHECK_NE(nullptr, encoder_);
  CHECK_NE(nullptr, index_);
  CHECK_NE(nullptr, potentiometer_);
  CHECK_NE(0, priority_);
  thread_ = ::std::thread(::std::ref(*this));
}

void InterruptEncoderAndPotentiometer::operator()() {
  ::aos::SetCurrentThreadName("IntEncPot_" +
                              ::std::to_string(potentiometer_->GetChannel()));

  index_->RequestInterrupts();
  index_->SetUpSourceEdge(true, false);

  ::aos::SetCurrentThreadRealtimePriority(priority_);

  InterruptableSensorBase::WaitResult result = InterruptableSensorBase::kBoth;
  while (run_) {
    result = index_->WaitForInterrupt(
        0.1, result != InterruptableSensorBase::kTimeout);
    if (result == InterruptableSensorBase::kTimeout) {
      continue;
    }

    {
      ::aos::MutexLocker locker(&mutex_);
      last_potentiometer_voltage_ = potentiometer_->GetVoltage();
      last_encoder_value_ = encoder_->GetRaw();
      ++index_posedge_count_;
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
