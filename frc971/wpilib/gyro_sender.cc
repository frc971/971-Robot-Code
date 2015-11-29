#include "frc971/wpilib/gyro_sender.h"

#include <inttypes.h>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"

#include "frc971/queues/gyro.q.h"

namespace frc971 {
namespace wpilib {

GyroSender::GyroSender() {}

void GyroSender::operator()() {
  ::aos::SetCurrentThreadName("Gyro");

  // Try to initialize repeatedly as long as we're supposed to be running.
  while (run_ && !gyro_.InitializeGyro()) {
    ::aos::time::SleepFor(::aos::time::Time::InMS(50));
  }
  LOG(INFO, "gyro initialized successfully\n");

  auto message = ::frc971::sensors::gyro_part_id.MakeMessage();
  message->uid = gyro_.ReadPartID();
  LOG_STRUCT(INFO, "gyro ID", *message);
  message.Send();

  // In radians, ready to send out.
  double angle = 0;

  int startup_cycles_left = 2 * kReadingRate;

  double zeroing_data[6 * kReadingRate];
  size_t zeroing_index = 0;
  bool zeroed = false;
  bool have_zeroing_data = false;
  double zero_offset = 0;

  ::aos::time::PhasedLoop phased_loop(
      ::aos::time::Time::FromRate(kReadingRate));
  // How many timesteps the next reading represents.
  int number_readings = 0;

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
    auto message = ::frc971::sensors::gyro_reading.MakeMessage();
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
      zeroing_data[zeroing_index] = new_angle;
      ++zeroing_index;
      if (zeroing_index >= sizeof(zeroing_data) / sizeof(zeroing_data[0])) {
        zeroing_index = 0;
        have_zeroing_data = true;
      }

      ::aos::joystick_state.FetchLatest();
      if (::aos::joystick_state.get() && ::aos::joystick_state->enabled &&
          have_zeroing_data) {
        zero_offset = 0;
        for (size_t i = 0; i < sizeof(zeroing_data) / sizeof(zeroing_data[0]);
             ++i) {
          zero_offset -= zeroing_data[i];
        }
        zero_offset /= sizeof(zeroing_data) / sizeof(zeroing_data[0]);
        LOG(INFO, "total zero offset %f\n", zero_offset);
        zeroed = true;
      }
    }
    number_readings = 0;
  }
}

}  // namespace wpilib
}  // namespace frc971
