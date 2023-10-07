#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_LED_INDICATOR_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_LED_INDICATOR_H_

#include "ctre/phoenix/led/CANdle.h"

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_output_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_position_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023_bot3::control_loops::superstructure {

class FlashCounter {
 public:
  FlashCounter(size_t flash_iterations) : flash_iterations_(flash_iterations) {}

  bool Flash() {
    if (counter_ % flash_iterations_ == 0) {
      flash_ = !flash_;
    }
    counter_++;
    return flash_;
  }

 private:
  size_t flash_iterations_;
  size_t counter_ = 0;
  bool flash_ = false;
};

class LedIndicator {
 public:
  LedIndicator(aos::EventLoop *event_loop);

  void DecideColor();

 private:
  static constexpr size_t kFlashIterations = 5;

  void DisplayLed(uint8_t r, uint8_t g, uint8_t b);

  ctre::phoenix::led::CANdle candle_{8, "rio"};

  aos::EventLoop *event_loop_;
  aos::Fetcher<frc971::control_loops::drivetrain::Output>
      drivetrain_output_fetcher_;
  aos::Fetcher<Status> superstructure_status_fetcher_;
  aos::Fetcher<Position> superstructure_position_fetcher_;
  aos::Fetcher<Goal> superstructure_goal_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics>
      server_statistics_fetcher_;
  aos::Fetcher<aos::message_bridge::ClientStatistics>
      client_statistics_fetcher_;
  aos::Fetcher<frc971::controls::LocalizerOutput> localizer_output_fetcher_;
  aos::Fetcher<frc971::sensors::GyroReading> gyro_reading_fetcher_;
  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  aos::monotonic_clock::time_point last_accepted_time_ =
      aos::monotonic_clock::min_time;

  aos::monotonic_clock::time_point startup_time_ =
      aos::monotonic_clock::min_time;

  FlashCounter flash_counter_{kFlashIterations};
};

}  // namespace y2023_bot3::control_loops::superstructure

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_LED_INDICATOR_H_
