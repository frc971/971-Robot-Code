#include "y2024/control_loops/superstructure/led_indicator.h"

namespace led = ctre::phoenix::led;
namespace chrono = std::chrono;

namespace y2024::control_loops::superstructure {

LedIndicator::LedIndicator(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      drivetrain_output_fetcher_(
          event_loop_->MakeFetcher<frc971::control_loops::drivetrain::Output>(
              "/drivetrain")),
      superstructure_status_fetcher_(
          event_loop_->MakeFetcher<Status>("/superstructure")),
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

  event_loop_->AddPhasedLoop([this](int) { DecideColor(); },
                             chrono::milliseconds(20));
  event_loop_->OnRun(
      [this]() { startup_time_ = event_loop_->monotonic_now(); });
}

// This method will be called once per scheduler run
void LedIndicator::DisplayLed(uint8_t r, uint8_t g, uint8_t b) {
  candle_.SetLEDs(static_cast<int>(r), static_cast<int>(g),
                  static_cast<int>(b));
}

bool DisconnectedIMUPiServer(
    const aos::message_bridge::ServerStatistics &server_statistics) {
  for (const auto *node_status : *server_statistics.connections()) {
    if (node_status->state() == aos::message_bridge::State::DISCONNECTED) {
      return true;
    }
  }

  return false;
}

bool DisconnectedIMUPiClient(
    const aos::message_bridge::ClientStatistics &client_statistics) {
  for (const auto *node_status : *client_statistics.connections()) {
    if (node_status->state() == aos::message_bridge::State::DISCONNECTED) {
      return true;
    }
  }

  return false;
}

bool OrinsDisconnected(
    const frc971::controls::LocalizerOutput &localizer_output) {
  if (!localizer_output.all_pis_connected()) {
    return true;
  } else {
    return false;
  }
}

void LedIndicator::DecideColor() {
  superstructure_status_fetcher_.Fetch();
  server_statistics_fetcher_.Fetch();
  drivetrain_output_fetcher_.Fetch();
  drivetrain_status_fetcher_.Fetch();
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

  // Estopped: Red
  if (superstructure_status_fetcher_.get() &&
      superstructure_status_fetcher_->estopped()) {
    DisplayLed(255, 0, 0);
    return;
  }

  // If the imu gyro readings are not being sent/updated recently.  Only do this
  // after we've been on for a bit.
  if (event_loop_->context().monotonic_event_time >
          startup_time_ + chrono::seconds(5) &&
      (!gyro_reading_fetcher_.get() ||
       gyro_reading_fetcher_.context().monotonic_event_time +
               frc971::controls::kLoopFrequency * 10 <
           event_loop_->monotonic_now() ||
       !gyro_reading_fetcher_->has_velocity())) {
    // Flash red/white
    if (flash_counter_.Flash()) {
      DisplayLed(255, 0, 0);
    } else {
      DisplayLed(255, 255, 255);
    }
    return;
  }

  if (localizer_output_fetcher_.get() == nullptr ||
      server_statistics_fetcher_.get() == nullptr ||
      client_statistics_fetcher_.get() == nullptr ||
      OrinsDisconnected(*localizer_output_fetcher_) ||
      DisconnectedIMUPiServer(*server_statistics_fetcher_) ||
      DisconnectedIMUPiClient(*client_statistics_fetcher_)) {
    // Flash red/green
    if (flash_counter_.Flash()) {
      DisplayLed(255, 0, 0);
    } else {
      DisplayLed(0, 255, 0);
    }

    return;
  }

  // Not zeroed: Yellow
  if ((superstructure_status_fetcher_.get() &&
       !superstructure_status_fetcher_->zeroed()) ||
      (drivetrain_status_fetcher_.get() &&
       !drivetrain_status_fetcher_->filters_ready())) {
    DisplayLed(255, 255, 0);
    return;
  }

  // Want to know when we have a note.
  //
  if (superstructure_status_fetcher_.get()) {
    // Check if there is a target that is in sight
    if (event_loop_->monotonic_now() <
        last_accepted_time_ + chrono::milliseconds(100)) {
      if (superstructure_status_fetcher_.get() != nullptr &&
          superstructure_status_fetcher_->shooter()->auto_aiming()) {
        DisplayLed(0, 0, 255);
        return;
      } else {
        DisplayLed(0, 255, 0);
        return;
      }
    }
  }
  DisplayLed(0, 0, 0);
}

}  // namespace y2024::control_loops::superstructure
