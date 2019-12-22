#include "frc971/wpilib/pdp_fetcher.h"

#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/logging/logging.h"
#include "frc971/wpilib/ahal/PowerDistributionPanel.h"
#include "frc971/wpilib/pdp_values_generated.h"

namespace frc971 {
namespace wpilib {

namespace chrono = ::std::chrono;

PDPFetcher::PDPFetcher(::aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      pdp_values_sender_(event_loop_->MakeSender<::frc971::PDPValues>("/aos")),
      pdp_(new frc::PowerDistributionPanel()) {
  event_loop->set_name("PDPFetcher");

  // SCHED_OTHER on purpose.
  event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                             chrono::milliseconds(20), chrono::milliseconds(4));
}

PDPFetcher::~PDPFetcher() {}

void PDPFetcher::Loop(int iterations) {
  if (iterations != 1) {
    AOS_LOG(DEBUG, "PDPFetcher skipped %d iterations\n", iterations - 1);
  }
  std::array<double, 16> currents;
  for (size_t i = 0; i < currents.size(); ++i) {
    currents[i] = pdp_->GetCurrent(i);
  }

  auto builder = pdp_values_sender_.MakeBuilder();
  flatbuffers::Offset<flatbuffers::Vector<double>> currents_offset =
      builder.fbb()->CreateVector(currents.begin(), currents.size());

  PDPValues::Builder pdp_builder = builder.MakeBuilder<PDPValues>();
  pdp_builder.add_voltage(pdp_->GetVoltage());
  pdp_builder.add_temperature(pdp_->GetTemperature());
  pdp_builder.add_power(pdp_->GetTotalPower());
  pdp_builder.add_currents(currents_offset);

  if (!builder.Send(pdp_builder.Finish())) {
    AOS_LOG(WARNING, "sending pdp values failed\n");
  }
}

}  // namespace wpilib
}  // namespace frc971
