#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <functional>
#include <mutex>
#include <thread>

#include "AnalogInput.h"
#include "Compressor.h"
#include "DigitalGlitchFilter.h"
#include "DriverStation.h"
#include "Encoder.h"
#include "Relay.h"
#include "Talon.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#undef ERROR

#include "aos/common/commonmath.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/stl_mutex.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/linux_code/init.h"

#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016_bot4/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/wpilib_interface.h"

#include "y2016_bot4/control_loops/drivetrain/drivetrain_base.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;

namespace y2016_bot4 {
namespace wpilib {

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2016_bot4::constants::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         ::y2016_bot4::constants::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

constexpr double kMaxDrivetrainEncoderPulsesPerSecond =
    5600.0 /* CIM free speed RPM */ * 12.0 / 54.0 *
    60.0 /* seconds per minute */ * 256.0 /* CPR */ * 4 /* edges per cycle */;

// Class to send position messages with sensor readings to our loops.
class SensorReader {
 public:
  SensorReader() {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    drivetrain_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 8.0 /* built-in tolerance */ /
                             kMaxDrivetrainEncoderPulsesPerSecond * 1e9 +
                         0.5));
  }

  // Drivetrain setters.
  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_encoder_filter_.Add(encoder.get());
    drivetrain_left_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("SensorReader");

    my_pid_ = getpid();
    ds_ = &DriverStation::GetInstance();

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        ::std::chrono::milliseconds(0));

    ::aos::SetCurrentThreadRealtimePriority(40);
    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(WARNING, "SensorReader skipped %d iterations\n", iterations - 1);
        }
      }
      RunIteration();
    }
  }

  void RunIteration() {
    ::frc971::wpilib::SendRobotState(my_pid_, ds_);

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }
  }

  void Quit() { run_ = false; }

 private:
  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter drivetrain_encoder_filter_, intake_encoder_filter_;
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_drivetrain_left_talon(::std::unique_ptr<Talon> t0,
                                 ::std::unique_ptr<Talon> t1) {
    drivetrain_left_talon_0_ = ::std::move(t0);
    drivetrain_left_talon_1_ = ::std::move(t1);
  }

  void set_drivetrain_right_talon(::std::unique_ptr<Talon> t0,
                                  ::std::unique_ptr<Talon> t1) {
    drivetrain_right_talon_0_ = ::std::move(t0);
    drivetrain_right_talon_1_ = ::std::move(t1);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    drivetrain_left_talon_0_->SetSpeed(-queue->left_voltage / 12.0);
    drivetrain_left_talon_1_->SetSpeed(-queue->left_voltage / 12.0);
    drivetrain_right_talon_0_->SetSpeed(queue->right_voltage / 12.0);
    drivetrain_right_talon_1_->SetSpeed(queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left_talon_0_->SetDisabled();
    drivetrain_right_talon_0_->SetDisabled();
    drivetrain_left_talon_1_->SetDisabled();
    drivetrain_right_talon_1_->SetDisabled();
  }

  ::std::unique_ptr<Talon> drivetrain_left_talon_0_, drivetrain_right_talon_0_,
      drivetrain_right_talon_1_, drivetrain_left_talon_1_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }

  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::frc971::wpilib::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader;

    reader.set_drivetrain_left_encoder(make_encoder(1));
    reader.set_drivetrain_right_encoder(make_encoder(0));
    ::std::thread reader_thread(::std::ref(reader));

    ::frc971::wpilib::GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_drivetrain_left_talon(
        ::std::unique_ptr<Talon>(new Talon(0)),
        ::std::unique_ptr<Talon>(new Talon(3)));
    drivetrain_writer.set_drivetrain_right_talon(
        ::std::unique_ptr<Talon>(new Talon(1)),
        ::std::unique_ptr<Talon>(new Talon(2)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    // Wait forever. Not much else to do...
    while (true) {
      const int r = select(0, nullptr, nullptr, nullptr, nullptr);
      if (r != 0) {
        PLOG(WARNING, "infinite select failed");
      } else {
        PLOG(WARNING, "infinite select succeeded??\n");
      }
    }

    LOG(ERROR, "Exiting WPILibRobot\n");

    joystick_sender.Quit();
    joystick_thread.join();
    pdp_fetcher.Quit();
    pdp_fetcher_thread.join();
    reader.Quit();
    reader_thread.join();
    gyro_sender.Quit();
    gyro_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2016_bot4

AOS_ROBOT_CLASS(::y2016_bot4::wpilib::WPILibRobot);
