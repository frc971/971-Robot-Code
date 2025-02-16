#include "ctre/phoenix/led/CANdle.h"

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "y2025/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2025/control_loops/superstructure/superstructure_output_generated.h"
#include "y2025/control_loops/superstructure/superstructure_position_generated.h"
#include "y2025/control_loops/superstructure/superstructure_status_generated.h"

namespace y2025::control_loops::superstructure {

class FlashCounter {
 public:
  uint8_t flash_iterations_ = 1;
  FlashCounter() {}

  bool Flash() {
    // TODO: confirm that 40 is the correct number of iterations to pass by
    // before next flash
    flashing_ = flash_iterations_ % 80 >= 40;
    return flashing_;
  }

  void IncrementFlashCounter() { flash_iterations_++; }

 private:
  bool flashing_ = false;
};

class LedIndicator {
 public:
  LedIndicator(aos::EventLoop *event_loop);
  static constexpr uint8_t candle_rio_bus_ = 8;
  ctre::phoenix::led::CANdle candle_{candle_rio_bus_, "rio"};

  /*
  Color Key:

  General/Miscellaneous:

  Red: stopped
  Flash red/white: imu is not connected
  Flash red/green: any orin is disconnected
  Pink: not zeroed

  Superstructure specific:

  Intaking: Orange/Off
  Moving: Flash yellow/off
  Spitting/Scoring: Purple
  */

  void CheckColor();

 private:
  void DisplayOnLED(uint8_t r, uint8_t g, uint8_t b);

  bool DisconnectedOrinClient(
      const aos::message_bridge::ClientStatistics &client_stats);

  bool DisconnectedOrinServer(
      const aos::message_bridge::ServerStatistics &server_stats);

  aos::EventLoop *event_loop_;
  aos::Fetcher<Status> superstructure_status_fetcher_;
  aos::Fetcher<Position> superstructure_position_fetcher_;
  aos::Fetcher<Goal> superstructure_goal_fetcher_;
  aos::Fetcher<frc971::sensors::GyroReading> gyro_reading_fetcher_;
  aos::Fetcher<frc971::control_loops::swerve::Status>
      drivetrain_status_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics>
      server_statistics_fetcher_;
  aos::Fetcher<aos::message_bridge::ClientStatistics>
      client_statistics_fetcher_;
  FlashCounter flash_counter_;
  aos::monotonic_clock::time_point startup_time_ =
      aos::monotonic_clock::min_time;

  bool gyro_last_had_velocity_ = true;
};

}  // namespace y2025::control_loops::superstructure