#include "y2025/control_loops/superstructure/led_indicator.h"

#include <chrono>

ABSL_FLAG(bool, use_orin1, true,
          "Use only the orin1 node instead of the imu node.");
using y2025::control_loops::superstructure::LedIndicator;

namespace chrono = std::chrono;

namespace y2025::control_loops::superstructure {

LedIndicator::LedIndicator(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      superstructure_status_fetcher_(
          event_loop_->MakeFetcher<Status>("/superstructure")),
      superstructure_position_fetcher_(
          event_loop_->MakeFetcher<Position>("/superstructure")),
      superstructure_goal_fetcher_(
          event_loop_->MakeFetcher<Goal>("/superstructure")),
      gyro_reading_fetcher_(
          event_loop_->MakeFetcher<frc971::sensors::GyroReading>(
              "/drivetrain")),
      drivetrain_status_fetcher_(
          event_loop_->MakeFetcher<frc971::control_loops::swerve::Status>(
              absl::GetFlag(FLAGS_use_orin1) ? "/orin1/drivetrain"
                                             : "/imu/drivetrain")),
      server_statistics_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/roborio/aos")),
      client_statistics_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ClientStatistics>(
              "/roborio/aos")),
      flash_counter_() {
  ctre::phoenix::led::CANdleConfiguration config;
  config.statusLedOffWhenActive = true;
  config.disableWhenLOS = false;
  config.brightnessScalar = 1.0;
  candle_.ConfigAllSettings(config, 0);

  event_loop_->AddPhasedLoop(
      [this](int) {
        flash_counter_.IncrementFlashCounter();
        CheckColor();
      },
      chrono::milliseconds(20));
  event_loop_->OnRun(
      [this]() { startup_time_ = event_loop_->monotonic_now(); });
}

void LedIndicator::DisplayOnLED(uint8_t r, uint8_t g, uint8_t b) {
  candle_.SetLEDs(r, g, b);
}

bool LedIndicator::DisconnectedOrinServer(
    const aos::message_bridge::ServerStatistics &server_statistics) {
  for (const auto *node_status : *server_statistics.connections()) {
    if (node_status->state() == aos::message_bridge::State::DISCONNECTED) {
      return true;
    }
  }
  return false;
}

bool LedIndicator::DisconnectedOrinClient(
    const aos::message_bridge::ClientStatistics &client_statistics) {
  for (const auto *node_status : *client_statistics.connections()) {
    if (node_status->state() == aos::message_bridge::State::DISCONNECTED) {
      return true;
    }
  }

  return false;
}

void LedIndicator::CheckColor() {
  superstructure_status_fetcher_.Fetch();
  superstructure_goal_fetcher_.Fetch();
  superstructure_position_fetcher_.Fetch();
  gyro_reading_fetcher_.Fetch();
  drivetrain_status_fetcher_.Fetch();
  server_statistics_fetcher_.Fetch();
  client_statistics_fetcher_.Fetch();

  // Stopped: Red
  if (superstructure_status_fetcher_.get() != nullptr &&
      superstructure_status_fetcher_->estopped()) {
    DisplayOnLED(255, 0, 0);
    return;
  }

  // 1+ orin pis not connected: red/green flash

  if (server_statistics_fetcher_.get() != nullptr &&
      client_statistics_fetcher_.get() != nullptr &&
      (DisconnectedOrinServer(*server_statistics_fetcher_) ||
       DisconnectedOrinClient(*client_statistics_fetcher_))) {
    if (flash_counter_.Flash()) {
      DisplayOnLED(255, 0, 0);
      return;
    } else {
      DisplayOnLED(0, 255, 0);
      return;
    }
  }

  // imu not connected: red/white flash
  if (event_loop_->context().monotonic_event_time >
          startup_time_ + chrono::seconds(5) &&
      (!gyro_reading_fetcher_.get() ||
       gyro_reading_fetcher_.context().monotonic_event_time +
               frc971::controls::kLoopFrequency * 10 <
           event_loop_->monotonic_now() ||
       !(gyro_reading_fetcher_->has_velocity() || gyro_last_had_velocity_))) {
    if (flash_counter_.Flash()) {
      DisplayOnLED(255, 0, 0);
      return;
    } else {
      DisplayOnLED(255, 255, 255);
      return;
    }
  }
  gyro_last_had_velocity_ = gyro_reading_fetcher_->has_velocity();
  // make it so gyro has to not have velocity twice in a row

  // Not zeroed: Pink
  if (superstructure_status_fetcher_.get() != nullptr &&
      !(superstructure_status_fetcher_->zeroed())) {
    DisplayOnLED(255, 20, 147);
    return;
  }

  // Scoring/Spitting : Purple

  if (superstructure_goal_fetcher_.get() != nullptr &&
      (superstructure_goal_fetcher_->pivot_goal() == PivotGoal::SCORE_L1 ||
       superstructure_goal_fetcher_->pivot_goal() == PivotGoal::SCORE_L2 ||
       superstructure_goal_fetcher_->pivot_goal() == PivotGoal::SCORE_L3 ||
       superstructure_goal_fetcher_->pivot_goal() == PivotGoal::SCORE_L4 ||
       superstructure_goal_fetcher_->elevator_goal() ==
           ElevatorGoal::SCORE_L1 ||
       superstructure_goal_fetcher_->elevator_goal() ==
           ElevatorGoal::SCORE_L2 ||
       superstructure_goal_fetcher_->elevator_goal() ==
           ElevatorGoal::SCORE_L3 ||
       superstructure_goal_fetcher_->elevator_goal() ==
           ElevatorGoal::SCORE_L4 ||
       superstructure_status_fetcher_->end_effector_state() ==
           EndEffectorStatus::SPITTING)) {
    DisplayOnLED(255, 140, 0);
    return;
  }

  // Intake: Flash Orange/Off

  if (superstructure_goal_fetcher_.get() != nullptr &&
      (superstructure_goal_fetcher_->end_effector_goal() ==
       EndEffectorGoal::INTAKE)) {
    if (flash_counter_.Flash()) {
      DisplayOnLED(255, 140, 0);
      return;
    } else {
      DisplayOnLED(0, 0, 0);
      return;
    }
  }

  // Moving: Flash yellow/off

  if (drivetrain_status_fetcher_.get() != nullptr &&
      ((abs(drivetrain_status_fetcher_->naive_estimator()->vx()) > 0.01) ||
       (abs(drivetrain_status_fetcher_->naive_estimator()->vy()) > 0.01) ||
       (abs(drivetrain_status_fetcher_->naive_estimator()->omega()) > 0.01))) {
    if (flash_counter_.Flash()) {
      DisplayOnLED(255, 255, 0);
      return;
    } else {
      DisplayOnLED(0, 0, 0);
      return;
    }
  }

  DisplayOnLED(0, 0, 0);
}

}  // namespace y2025::control_loops::superstructure
