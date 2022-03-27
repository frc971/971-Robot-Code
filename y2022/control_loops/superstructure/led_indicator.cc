#include "y2022/control_loops/superstructure/led_indicator.h"

namespace led = ctre::phoenix::led;

namespace y2022::control_loops::superstructure {

LedIndicator::LedIndicator(aos::EventLoop *event_loop)
    : drivetrain_output_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Output>(
              "/drivetrain")),
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<Status>("/superstructure")),
      server_statistics_fetcher_(
          event_loop->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/roborio/aos")),
      client_statistics_fetcher_(
          event_loop->MakeFetcher<aos::message_bridge::ClientStatistics>(
              "/roborio/aos")) {
  led::CANdleConfiguration config;
  config.statusLedOffWhenActive = true;
  config.disableWhenLOS = false;
  config.brightnessScalar = 1.0;
  candle_.ConfigAllSettings(config, 0);

  event_loop->AddPhasedLoop([&](int) { DecideColor(); },
                            std::chrono::milliseconds(20));
}

// This method will be called once per scheduler run
void LedIndicator::DisplayLed(uint8_t r, uint8_t g, uint8_t b) {
  candle_.SetLEDs(static_cast<int>(r), static_cast<int>(g),
                  static_cast<int>(b));
}

namespace {
bool DisconnectedPiServer(
    const aos::message_bridge::ServerStatistics &server_stats) {
  for (const auto *pi_server_status : *server_stats.connections()) {
    if (pi_server_status->state() == aos::message_bridge::State::DISCONNECTED &&
        pi_server_status->node()->name()->string_view() != "logger") {
      return true;
    }
  }
  return false;
}

bool DisconnectedPiClient(
    const aos::message_bridge::ClientStatistics &client_stats) {
  for (const auto *pi_client_status : *client_stats.connections()) {
    if (pi_client_status->state() == aos::message_bridge::State::DISCONNECTED &&
        pi_client_status->node()->name()->string_view() != "logger") {
      return true;
    }
  }
  return false;
}

bool DrivingFast(
    const frc971::control_loops::drivetrain::Output &drivetrain_out) {
  return (drivetrain_out.left_voltage() >= 11.5 ||
          drivetrain_out.right_voltage() >= 11.5);
}
}  // namespace

void LedIndicator::DecideColor() {
  superstructure_status_fetcher_.Fetch();
  server_statistics_fetcher_.Fetch();
  drivetrain_output_fetcher_.Fetch();
  client_statistics_fetcher_.Fetch();

  // Estopped
  if (superstructure_status_fetcher_.get() &&
      superstructure_status_fetcher_->estopped()) {
    DisplayLed(255, 0, 0);
    return;
  }

  // Not zeroed
  if (superstructure_status_fetcher_.get() &&
      !superstructure_status_fetcher_->zeroed()) {
    DisplayLed(255, 255, 0);
    return;
  }

  // Pi disconnected
  if ((server_statistics_fetcher_.get() &&
       DisconnectedPiServer(*server_statistics_fetcher_)) ||
      (client_statistics_fetcher_.get() &&
       DisconnectedPiClient(*client_statistics_fetcher_))) {
    if (disconnected_flash_) {
      DisplayLed(255, 0, 0);
    } else {
      DisplayLed(0, 255, 0);
    }

    if (disconnected_counter_ % kFlashIterations == 0) {
      disconnected_flash_ = !disconnected_flash_;
    }
    disconnected_counter_++;
    return;
  }

  // Driving fast
  if (drivetrain_output_fetcher_.get() &&
      DrivingFast(*drivetrain_output_fetcher_)) {
    DisplayLed(138, 43, 226);
    return;
  }

  // Statemachine
  if (superstructure_status_fetcher_.get()) {
    switch (superstructure_status_fetcher_->state()) {
      case (SuperstructureState::IDLE):
        DisplayLed(0, 0, 0);
        break;
      case (SuperstructureState::TRANSFERRING):
        DisplayLed(0, 0, 255);
        break;
      case (SuperstructureState::LOADING):
        DisplayLed(255, 255, 255);
        break;
      case (SuperstructureState::LOADED):
        if (!superstructure_status_fetcher_->ready_to_fire()) {
          DisplayLed(255, 140, 0);
        } else if (superstructure_status_fetcher_->front_intake_has_ball() ||
                   superstructure_status_fetcher_->back_intake_has_ball()) {
          DisplayLed(165, 42, 42);
        } else {
          DisplayLed(0, 255, 0);
        }
        break;
      case (SuperstructureState::SHOOTING):
        if (!superstructure_status_fetcher_->flippers_open()) {
          DisplayLed(255, 105, 180);
        } else {
          DisplayLed(0, 255, 255);
        }
        break;
    }
    return;
  }
}

}  // namespace y2022::control_loops::superstructure
