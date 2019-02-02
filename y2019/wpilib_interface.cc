#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/make_unique.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/wpilib_interface.h"
#include "frc971/wpilib/wpilib_robot_base.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
using aos::make_unique;

namespace y2019 {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

// TODO(brian): Use ::std::max instead once we have C++14 so that can be
// constexpr.
template <typename T>
constexpr T max(T a, T b) {
  return (a > b) ? a : b;
}

template <typename T, typename... Rest>
constexpr T max(T a, T b, T c, Rest... rest) {
  return max(max(a, b), c, rest...);
}

double drivetrain_translate(int32_t in) {
  return ((static_cast<double>(in)
           /* / Values::kDrivetrainEncoderCountsPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius*/));
}

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) /* / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius*/));
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(/*Values::kMaxDrivetrainEncoderPulsesPerSecond(),
        Values::kMaxIntakeMotorEncoderPulsesPerSecond()*/ 1.0, 1.0);
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(/*Values::kMaxProximalEncoderPulsesPerSecond(),
        Values::kMaxDistalEncoderPulsesPerSecond()*/ 1.0, 1.0);
static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader() {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    fast_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxFastEncoderPulsesPerSecond * 1e9 +
                         0.5));
    medium_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxMediumEncoderPulsesPerSecond * 1e9 +
                         0.5));
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  void RunIteration() override {
    ::frc971::wpilib::SendRobotState(my_pid_);

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());

      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->right_speed = -drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();
  }
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_drivetrain_left_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_left_victor_ = ::std::move(t);
  }

  void set_drivetrain_right_victor(::std::unique_ptr<::frc::VictorSP> t) {
    drivetrain_right_victor_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    drivetrain_left_victor_->SetSpeed(
        ::aos::Clip(queue->left_voltage, -12.0, 12.0) / 12.0);
    drivetrain_right_victor_->SetSpeed(
        ::aos::Clip(-queue->right_voltage, -12.0, 12.0) / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left_victor_->SetDisabled();
    drivetrain_right_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> drivetrain_left_victor_,
      drivetrain_right_victor_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::frc971::wpilib::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader;

    // TODO(Sabina): Update port numbers(Sensors and Victors)
    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    reader.set_pwm_trigger(make_unique<frc::DigitalInput>(25));

    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    auto imu_trigger = make_unique<frc::DigitalInput>(5);
    ::frc971::wpilib::ADIS16448 imu(frc::SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.SetDummySPI(frc::SPI::Port::kOnboardCS2);
    auto imu_reset = make_unique<frc::DigitalOutput>(6);
    imu.set_reset(imu_reset.get());
    ::std::thread imu_thread(::std::ref(imu));

    // While as of 2/9/18 the drivetrain Victors are SPX, it appears as though
    // they are identical, as far as DrivetrainWriter is concerned, to the SP
    // variety so all the Victors are written as SPs.

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_drivetrain_left_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    drivetrain_writer.set_drivetrain_right_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)));
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
    imu.Quit();
    imu_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2019

AOS_ROBOT_CLASS(::y2019::wpilib::WPILibRobot);
