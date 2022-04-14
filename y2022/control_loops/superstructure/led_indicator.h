#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_LED_INDICATOR_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_LED_INDICATOR_H_

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "ctre/phoenix/led/CANdle.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "y2022/control_loops/superstructure/superstructure_output_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/localizer/localizer_output_generated.h"

namespace y2022::control_loops::superstructure {

class LedIndicator {
 public:
  LedIndicator(aos::EventLoop *event_loop);

  // Colors in order of priority:
  //
  // Red: estopped
  // Yellow: not zeroed
  // Flash red/white: imu disconnected
  // Flash red/green: pi disconnected
  // Purple: driving fast
  //
  // Statemachine:
  // IDLE:
  //    Off
  // TRANSFERRING:
  //    Blue
  // LOADING:
  //    White
  // LOADED:
  //    Green: ready to fire
  //    Brown: intaked another ball
  //    Orange: loaded
  // SHOOTING:
  //    Pink: flippers opening
  //    Cyan: superstructure shooting
  void DecideColor();

 private:
  static constexpr size_t kFlashIterations = 5;

  void DisplayLed(uint8_t r, uint8_t g, uint8_t b);

  ctre::phoenix::led::CANdle candle_{0, ""};

  aos::EventLoop *event_loop_;
  aos::Fetcher<frc971::control_loops::drivetrain::Output>
      drivetrain_output_fetcher_;
  aos::Fetcher<Status> superstructure_status_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics>
      server_statistics_fetcher_;
  aos::Fetcher<aos::message_bridge::ClientStatistics>
      client_statistics_fetcher_;
  aos::Fetcher<frc971::controls::LocalizerOutput> localizer_output_fetcher_;
  aos::Fetcher<frc971::sensors::GyroReading> gyro_reading_fetcher_;

  size_t last_accepted_count_ = 0;
  aos::monotonic_clock::time_point last_accepted_time_ =
      aos::monotonic_clock::min_time;
  size_t imu_counter_ = 0;
  bool imu_flash_ = false;
  size_t disconnected_counter_ = 0;
  bool disconnected_flash_ = false;
};

}  // namespace y2022::control_loops::superstructure

#endif  // Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_LED_INDICATOR_H_
