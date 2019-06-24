#include "frc971/wpilib/gyro_sender.h"

#include <fcntl.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>

#include "aos/events/event-loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/robot_state/robot_state.q.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"

#include "frc971/queues/gyro.q.h"
#include "frc971/zeroing/averager.h"

namespace frc971 {
namespace wpilib {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

GyroSender::GyroSender(::aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      joystick_state_fetcher_(event_loop_->MakeFetcher<::aos::JoystickState>(
          ".aos.joystick_state")),
      uid_sender_(event_loop_->MakeSender<::frc971::sensors::Uid>(
          ".frc971.sensors.gyro_part_id")),
      gyro_reading_sender_(
          event_loop_->MakeSender<::frc971::sensors::GyroReading>(
              ".frc971.sensors.gyro_reading")) {
  PCHECK(
      system("ps -ef | grep '\\[spi0\\]' | awk '{print $1}' | xargs chrt -f -p "
             "33") == 0);
}

void GyroSender::operator()() {
  ::aos::SetCurrentThreadName("Gyro");

  // Try to initialize repeatedly as long as we're supposed to be running.
  while (run_ && !gyro_.InitializeGyro()) {
    ::std::this_thread::sleep_for(::std::chrono::milliseconds(50));
  }
  LOG(INFO, "gyro initialized successfully\n");

  auto message = uid_sender_.MakeMessage();
  message->uid = gyro_.ReadPartID();
  LOG_STRUCT(INFO, "gyro ID", *message);
  message.Send();

  // In radians, ready to send out.
  double angle = 0;

  int startup_cycles_left = 2 * kReadingRate;

  zeroing::Averager<double, 6 * kReadingRate> zeroing_data;
  bool zeroed = false;
  double zero_offset = 0;

  ::aos::SetCurrentThreadRealtimePriority(33);

  ::aos::time::PhasedLoop phased_loop(::aos::time::FromRate(kReadingRate),
                                      event_loop_->monotonic_now(),
                                      chrono::milliseconds(4));
  // How many timesteps the next reading represents.
  int number_readings = 0;

  ::aos::SetCurrentThreadRealtimePriority(33);

  while (run_) {
    number_readings += phased_loop.SleepUntilNext();

    const uint32_t result = gyro_.GetReading();
    if (result == 0) {
      LOG(WARNING, "normal gyro read failed\n");
      continue;
    }
    switch (gyro_.ExtractStatus(result)) {
      case 0:
        LOG(WARNING, "gyro says data is bad\n");
        continue;
      case 1:
        break;
      default:
        LOG(WARNING, "gyro gave weird status 0x%" PRIx8 "\n",
            gyro_.ExtractStatus(result));
        continue;
    }
    if (gyro_.ExtractErrors(result) != 0) {
      const uint8_t errors = gyro_.ExtractErrors(result);
      if (errors & (1 << 6)) {
        LOG(WARNING, "gyro gave PLL error\n");
      }
      if (errors & (1 << 5)) {
        LOG(WARNING, "gyro gave quadrature error\n");
      }
      if (errors & (1 << 4)) {
        LOG(WARNING, "gyro gave non-volatile memory error\n");
      }
      if (errors & (1 << 3)) {
        LOG(WARNING, "gyro gave volatile memory error\n");
      }
      if (errors & (1 << 2)) {
        LOG(WARNING, "gyro gave power error\n");
      }
      if (errors & (1 << 1)) {
        LOG(WARNING, "gyro gave continuous self-test error\n");
      }
      if (errors & 1) {
        LOG(WARNING, "gyro gave unexpected self-test mode\n");
      }
      continue;
    }

    if (startup_cycles_left > 0) {
      --startup_cycles_left;
      continue;
    }

    const double angle_rate = gyro_.ExtractAngle(result);
    const double new_angle = angle_rate / static_cast<double>(kReadingRate);
    auto message = gyro_reading_sender_.MakeMessage();
    if (zeroed) {
      angle += (new_angle + zero_offset) * number_readings;
      message->angle = angle;
      message->velocity = angle_rate + zero_offset * kReadingRate;
      LOG_STRUCT(DEBUG, "sending", *message);
      message.Send();
    } else {
      // TODO(brian): Don't break without 6 seconds of standing still before
      // enabling. Ideas:
      //   Don't allow driving until we have at least some data?
      //   Some kind of indicator light?
      {
        message->angle = new_angle;
        message->velocity = angle_rate;
        LOG_STRUCT(DEBUG, "collected while zeroing", *message);
        message->angle = 0.0;
        message->velocity = 0.0;
        message.Send();
      }
      zeroing_data.AddData(new_angle);

      joystick_state_fetcher_.Fetch();
      if (joystick_state_fetcher_.get() && joystick_state_fetcher_->enabled &&
          zeroing_data.full()) {
        zero_offset = -zeroing_data.GetAverage();
        LOG(INFO, "total zero offset %f\n", zero_offset);
        zeroed = true;
      }
    }
    number_readings = 0;
  }
}

}  // namespace wpilib
}  // namespace frc971
