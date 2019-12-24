#include "frc971/wpilib/gyro_sender.h"

#include <fcntl.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/time/time.h"

#include "frc971/queues/gyro_generated.h"
#include "frc971/zeroing/averager.h"

namespace frc971 {
namespace wpilib {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

GyroSender::GyroSender(::aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      joystick_state_fetcher_(
          event_loop_->MakeFetcher<aos::RobotState>("/aos")),
      uid_sender_(event_loop_->MakeSender<frc971::sensors::Uid>("/drivetrain")),
      gyro_reading_sender_(
          event_loop_->MakeSender<frc971::sensors::GyroReading>(
              "/drivetrain")) {
  AOS_PCHECK(
      system("ps -ef | grep '\\[spi0\\]' | awk '{print $1}' | xargs chrt -f -p "
             "33") == 0);
  event_loop->set_name("Gyro");
  event_loop_->SetRuntimeRealtimePriority(33);

  // TODO(austin): This should be synchronized with SensorReader...  Pull out
  // the sync logic and re-use it here.
  event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                             ::aos::time::FromRate(kReadingRate),
                             chrono::milliseconds(4));
}

void GyroSender::Loop(const int iterations) {
  switch (state_) {
    case State::INITIALIZING: {
      const monotonic_clock::time_point monotonic_now =
          event_loop_->monotonic_now();
      if (last_initialize_time_ + chrono::milliseconds(50) < monotonic_now) {
        if (gyro_.InitializeGyro()) {
          state_ = State::RUNNING;
          AOS_LOG(INFO, "gyro initialized successfully\n");

          auto builder = uid_sender_.MakeBuilder();
          builder.Send(
              frc971::sensors::CreateUid(*builder.fbb(), gyro_.ReadPartID()));
        }
        last_initialize_time_ = monotonic_now;
      }
    } break;
    case State::RUNNING: {
      const uint32_t result = gyro_.GetReading();
      if (result == 0) {
        AOS_LOG(WARNING, "normal gyro read failed\n");
        return;
      }
      switch (gyro_.ExtractStatus(result)) {
        case 0:
          AOS_LOG(WARNING, "gyro says data is bad\n");
          return;
        case 1:
          break;
        default:
          AOS_LOG(WARNING, "gyro gave weird status 0x%" PRIx8 "\n",
                  gyro_.ExtractStatus(result));
          return;
      }
      if (gyro_.ExtractErrors(result) != 0) {
        const uint8_t errors = gyro_.ExtractErrors(result);
        if (errors & (1 << 6)) {
          AOS_LOG(WARNING, "gyro gave PLL error\n");
        }
        if (errors & (1 << 5)) {
          AOS_LOG(WARNING, "gyro gave quadrature error\n");
        }
        if (errors & (1 << 4)) {
          AOS_LOG(WARNING, "gyro gave non-volatile memory error\n");
        }
        if (errors & (1 << 3)) {
          AOS_LOG(WARNING, "gyro gave volatile memory error\n");
        }
        if (errors & (1 << 2)) {
          AOS_LOG(WARNING, "gyro gave power error\n");
        }
        if (errors & (1 << 1)) {
          AOS_LOG(WARNING, "gyro gave continuous self-test error\n");
        }
        if (errors & 1) {
          AOS_LOG(WARNING, "gyro gave unexpected self-test mode\n");
        }
        return;
      }

      if (startup_cycles_left_ > 0) {
        --startup_cycles_left_;
        return;
      }

      const double angle_rate = gyro_.ExtractAngle(result);
      const double new_angle = angle_rate / static_cast<double>(kReadingRate);
      auto builder = gyro_reading_sender_.MakeBuilder();
      if (zeroed_) {
        angle_ += (new_angle + zero_offset_) * iterations;
        sensors::GyroReading::Builder gyro_builder =
            builder.MakeBuilder<sensors::GyroReading>();
        gyro_builder.add_angle(angle_);
        gyro_builder.add_velocity(angle_rate + zero_offset_ * kReadingRate);
        builder.Send(gyro_builder.Finish());
      } else {
        // TODO(brian): Don't break without 6 seconds of standing still before
        // enabling. Ideas:
        //   Don't allow driving until we have at least some data?
        //   Some kind of indicator light?
        {
          sensors::GyroReading::Builder gyro_builder =
              builder.MakeBuilder<sensors::GyroReading>();
          gyro_builder.add_angle(0.0);
          gyro_builder.add_velocity(0.0);
          builder.Send(gyro_builder.Finish());
        }
        zeroing_data_.AddData(new_angle);

        joystick_state_fetcher_.Fetch();
        if (joystick_state_fetcher_.get() &&
            joystick_state_fetcher_->outputs_enabled() &&
            zeroing_data_.full()) {
          zero_offset_ = -zeroing_data_.GetAverage();
          AOS_LOG(INFO, "total zero offset %f\n", zero_offset_);
          zeroed_ = true;
        }
      }
    } break;
  }
}

}  // namespace wpilib
}  // namespace frc971
