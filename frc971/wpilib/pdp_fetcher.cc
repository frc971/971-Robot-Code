#include "frc971/wpilib/pdp_fetcher.h"

#include <chrono>

#include "aos/events/event-loop.h"
#include "aos/init.h"
#include "aos/logging/queue_logging.h"
#include "frc971/wpilib/ahal/PowerDistributionPanel.h"
#include "frc971/wpilib/pdp_values.q.h"

namespace frc971 {
namespace wpilib {

namespace chrono = ::std::chrono;

PDPFetcher::PDPFetcher(::aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      pdp_values_sender_(
          event_loop_->MakeSender<::frc971::PDPValues>(".frc971.pdp_values")),
      pdp_(new frc::PowerDistributionPanel()) {
  event_loop_->set_name("PDPFetcher");

  // SCHED_OTHER on purpose.
  event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                             chrono::milliseconds(20), chrono::milliseconds(4));
}

PDPFetcher::~PDPFetcher() {}

void PDPFetcher::Loop(int iterations) {
  if (iterations != 1) {
    LOG(DEBUG, "PDPFetcher skipped %d iterations\n", iterations - 1);
  }
  auto message = pdp_values_sender_.MakeMessage();
  message->voltage = pdp_->GetVoltage();
  message->temperature = pdp_->GetTemperature();
  message->power = pdp_->GetTotalPower();
  for (int i = 0; i < 16; ++i) {
    message->currents[i] = pdp_->GetCurrent(i);
  }
  LOG_STRUCT(DEBUG, "got", *message);
  if (!message.Send()) {
    LOG(WARNING, "sending pdp values failed\n");
  }
}

}  // namespace wpilib
}  // namespace frc971
