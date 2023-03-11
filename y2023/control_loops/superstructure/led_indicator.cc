#include "y2023/control_loops/superstructure/led_indicator.h"

namespace led = ctre::phoenix::led;

namespace y2023::control_loops::superstructure {

LedIndicator::LedIndicator(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      drivetrain_output_fetcher_(
          event_loop_->MakeFetcher<frc971::control_loops::drivetrain::Output>(
              "/drivetrain")),
      superstructure_status_fetcher_(
          event_loop_->MakeFetcher<Status>("/superstructure")),
      superstructure_position_fetcher_(
          event_loop_->MakeFetcher<Position>("/superstructure")),
      server_statistics_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/roborio/aos")),
      client_statistics_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ClientStatistics>(
              "/roborio/aos")),
      localizer_output_fetcher_(
          event_loop_->MakeFetcher<frc971::controls::LocalizerOutput>(
              "/localizer")),
      gyro_reading_fetcher_(
          event_loop_->MakeFetcher<frc971::sensors::GyroReading>(
              "/drivetrain")),
      drivetrain_status_fetcher_(
          event_loop_->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")) {
  led::CANdleConfiguration config;
  config.statusLedOffWhenActive = true;
  config.disableWhenLOS = false;
  config.brightnessScalar = 1.0;
  candle_.ConfigAllSettings(config, 0);

  event_loop_->AddPhasedLoop([&](int) { DecideColor(); },
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
}  // namespace

void LedIndicator::DecideColor() {
  superstructure_status_fetcher_.Fetch();
  superstructure_position_fetcher_.Fetch();
  server_statistics_fetcher_.Fetch();
  drivetrain_output_fetcher_.Fetch();
  client_statistics_fetcher_.Fetch();
  gyro_reading_fetcher_.Fetch();
  localizer_output_fetcher_.Fetch();

  if (localizer_output_fetcher_.get()) {
    if (localizer_output_fetcher_->image_accepted_count() !=
        last_accepted_count_) {
      last_accepted_count_ = localizer_output_fetcher_->image_accepted_count();
      last_accepted_time_ = event_loop_->monotonic_now();
    }
  }

  // Estopped
  if (superstructure_status_fetcher_.get() &&
      superstructure_status_fetcher_->estopped()) {
    DisplayLed(255, 0, 0);
    return;
  }

  // Not zeroed
  if (superstructure_status_fetcher_.get() &&
      !superstructure_status_fetcher_->zeroed()) {
    DisplayLed(255, 0, 255);
    return;
  }

  // If the imu gyro readings are not being sent/updated recently
  if (!gyro_reading_fetcher_.get() ||
      gyro_reading_fetcher_.context().monotonic_event_time <
          event_loop_->monotonic_now() -
              frc971::controls::kLoopFrequency * 10) {
    if (flash_counter_.Flash()) {
      DisplayLed(255, 0, 0);
    } else {
      DisplayLed(255, 255, 255);
    }
    return;
  }

  // Pi disconnected
  if ((server_statistics_fetcher_.get() &&
       DisconnectedPiServer(*server_statistics_fetcher_)) ||
      (client_statistics_fetcher_.get() &&
       DisconnectedPiClient(*client_statistics_fetcher_))) {
    if (flash_counter_.Flash()) {
      DisplayLed(255, 0, 0);
    } else {
      DisplayLed(0, 255, 0);
    }

    return;
  }

  if (superstructure_status_fetcher_.get()) {
    // Check if end effector is intaking.
    if (superstructure_status_fetcher_->end_effector_state() ==
        EndEffectorState::INTAKING) {
      if (flash_counter_.Flash()) {
        DisplayLed(255, 165, 0);
      } else {
        DisplayLed(0, 0, 0);
      }

      return;
    }
    // Check if end effector is spitting.
    if (superstructure_status_fetcher_->end_effector_state() ==
        EndEffectorState::SPITTING) {
      if (flash_counter_.Flash()) {
        DisplayLed(0, 255, 0);
      } else {
        DisplayLed(0, 0, 0);
      }

      return;
    }

    // Check the if there is a cone in the end effector.
    if (superstructure_status_fetcher_->game_piece() ==
            vision::Class::CONE_UP ||
        superstructure_status_fetcher_->game_piece() ==
            vision::Class::CONE_DOWN) {
      DisplayLed(255, 255, 0);
      return;
    }
    // Check if the cube beam break is triggered.
    if (superstructure_status_fetcher_->game_piece() == vision::Class::CUBE) {
      DisplayLed(138, 43, 226);
      return;
    }

    // Check if there is a target that is in sight
    if (drivetrain_status_fetcher_->line_follow_logging()->have_target()) {
      DisplayLed(255, 165, 0);
      return;
    }

    if (event_loop_->monotonic_now() <
        last_accepted_time_ + std::chrono::milliseconds(500)) {
      DisplayLed(0, 0, 255);
      return;
    }

    return;
  }
}

}  // namespace y2023::control_loops::superstructure
