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
#include "aos/robot_state/robot_state.q.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "third_party/Phoenix-frc-lib/cpp/include/ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "y2019/constants.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::y2019::control_loops::superstructure::superstructure_queue;
using ::y2019::constants::Values;
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
  return ((static_cast<double>(in) /
           Values::kDrivetrainEncoderCountsPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

double elevator_pot_translate(double voltage) {
  return voltage * Values::kElevatorPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double wrist_pot_translate(double voltage) {
  return voltage * Values::kWristPotRatio() * (5.0 /*turns*/ / 5.0 /*volts*/) *
         (2 * M_PI /*radians*/);
}

double stilts_pot_translate(double voltage) {
  return voltage * Values::kStiltsPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(Values::kMaxDrivetrainEncoderPulsesPerSecond(),
        Values::kMaxIntakeEncoderPulsesPerSecond());
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(Values::kMaxElevatorEncoderPulsesPerSecond(),
        Values::kMaxWristEncoderPulsesPerSecond());

static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader() {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    UpdateMediumEncoderFilterHz(kMaxMediumEncoderPulsesPerSecond);
  }

  // Elevator

  void set_elevator_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    elevator_encoder_.set_encoder(::std::move(encoder));
  }

  void set_elevator_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    elevator_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_elevator_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    elevator_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Intake

  void set_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  // Wrist

  void set_wrist_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
  }

  void set_wrist_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    wrist_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_wrist_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    wrist_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Stilts

  void set_stilts_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    stilts_encoder_.set_encoder(::std::move(encoder));
  }

  void set_stilts_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    stilts_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_stilts_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    stilts_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Vacuum pressure sensor
  void set_vacuum_sensor(int port) {
    vacuum_sensor_ = make_unique<frc::AnalogInput>(port);
  }

  void RunIteration() override {
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
    const auto values = constants::GetValues();

    {
      auto superstructure_message = superstructure_queue.position.MakeMessage();

      // Elevator
      CopyPosition(elevator_encoder_, &superstructure_message->elevator,
                   Values::kElevatorEncoderCountsPerRevolution(),
                   Values::kElevatorEncoderRatio(), elevator_pot_translate,
                   false, values.elevator.potentiometer_offset);
      // Intake
      CopyPosition(intake_encoder_, &superstructure_message->intake_joint,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), false);

      // Wrist
      CopyPosition(wrist_encoder_, &superstructure_message->wrist,
                   Values::kWristEncoderCountsPerRevolution(),
                   Values::kWristEncoderRatio(), wrist_pot_translate, false,
                   values.wrist.potentiometer_offset);

      // Stilts
      CopyPosition(stilts_encoder_, &superstructure_message->stilts,
                   Values::kStiltsEncoderCountsPerRevolution(),
                   Values::kStiltsEncoderRatio(), stilts_pot_translate, false,
                   values.stilts.potentiometer_offset);

      // Suction
      constexpr float kMinVoltage = 0.5;
      constexpr float kMaxVoltage = 2.1;
      superstructure_message->suction_pressure =
          (vacuum_sensor_->GetVoltage() - kMinVoltage) /
          (kMaxVoltage - kMinVoltage);

      superstructure_message.Send();
    }
  }

 private:
  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer elevator_encoder_,
      wrist_encoder_, stilts_encoder_;

  ::std::unique_ptr<frc::AnalogInput> vacuum_sensor_;

  ::frc971::wpilib::AbsoluteEncoder intake_encoder_;
  // TODO(sabina): Add wrist and elevator hall effects.
};

class SuperstructureWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_elevator_victor(::std::unique_ptr<::frc::VictorSP> t) {
    elevator_victor_ = ::std::move(t);
  }

  void set_suction_victor(::std::unique_ptr<::frc::VictorSP> t) {
    suction_victor_ = ::std::move(t);
  }

  void set_intake_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_victor_ = ::std::move(t);
  }

  void set_wrist_victor(::std::unique_ptr<::frc::VictorSP> t) {
    wrist_victor_ = ::std::move(t);
  }

  void set_stilts_victor(::std::unique_ptr<::frc::VictorSP> t) {
    stilts_victor_ = ::std::move(t);
  }

 private:
  void Read() override {
    ::y2019::control_loops::superstructure::superstructure_queue.output
        .FetchAnother();
  }

  void Write() override {
    auto &queue =
        ::y2019::control_loops::superstructure::superstructure_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    elevator_victor_->SetSpeed(::aos::Clip(queue->elevator_voltage,
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                               12.0);

    intake_victor_->SetSpeed(::aos::Clip(queue->intake_joint_voltage,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    wrist_victor_->SetSpeed(::aos::Clip(-queue->wrist_voltage,
                                        -kMaxBringupPower, kMaxBringupPower) /
                            12.0);

    stilts_victor_->SetSpeed(::aos::Clip(queue->stilts_voltage,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    ::aos::robot_state.FetchLatest();
    const double battery_voltage =
        ::aos::robot_state.get() ? ::aos::robot_state->voltage_battery : 12.0;

    // Throw a fast low pass filter on the battery voltage so we don't respond
    // too fast to noise.
    filtered_battery_voltage_ =
        0.5 * filtered_battery_voltage_ + 0.5 * battery_voltage;

    suction_victor_->SetSpeed(::aos::Clip(
        queue->pump_voltage / filtered_battery_voltage_, -1.0, 1.0));
  }

  void Stop() override {
    LOG(WARNING, "Superstructure output too old.\n");

    elevator_victor_->SetDisabled();
    intake_victor_->SetDisabled();
    wrist_victor_->SetDisabled();
    stilts_victor_->SetDisabled();
    suction_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> elevator_victor_, intake_victor_,
      wrist_victor_, stilts_victor_, suction_victor_;

  double filtered_battery_voltage_ = 12.0;
};

class SolenoidWriter {
 public:
  SolenoidWriter()
      : superstructure_(
            ".y2019.control_loops.superstructure.superstructure_queue.output") {
  }

  void set_big_suction_cup(int index0, int index1) {
    big_suction_cup0_ = pcm_.MakeSolenoid(index0);
    big_suction_cup1_ = pcm_.MakeSolenoid(index1);
  }
  void set_small_suction_cup(int index0, int index1) {
    small_suction_cup0_ = pcm_.MakeSolenoid(index0);
    small_suction_cup1_ = pcm_.MakeSolenoid(index1);
  }

  void set_intake_roller_talon(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonSRX> t) {
    intake_rollers_talon_ = ::std::move(t);
    intake_rollers_talon_->ConfigContinuousCurrentLimit(20.0, 0);
    intake_rollers_talon_->EnableCurrentLimit(true);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(20),
                                        ::std::chrono::milliseconds(1));

    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      {
        superstructure_.FetchLatest();
        if (superstructure_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *superstructure_);

          big_suction_cup0_->Set(!superstructure_->intake_suction_top);
          big_suction_cup1_->Set(!superstructure_->intake_suction_top);
          small_suction_cup0_->Set(!superstructure_->intake_suction_bottom);
          small_suction_cup1_->Set(!superstructure_->intake_suction_bottom);

          intake_rollers_talon_->Set(
              ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
              ::aos::Clip(superstructure_->intake_roller_voltage,
                          -kMaxBringupPower, kMaxBringupPower) /
                  12.0);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;

        pcm_.Flush();
        to_log.read_solenoids = pcm_.GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }
    }
  }

  void Quit() { run_ = false; }

 private:
  ::frc971::wpilib::BufferedPcm pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> big_suction_cup0_,
      big_suction_cup1_, small_suction_cup0_, small_suction_cup1_;

  ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonSRX>
      intake_rollers_talon_;

  ::aos::Queue<
      ::y2019::control_loops::superstructure::SuperstructureQueue::Output>
      superstructure_;

  ::std::atomic<bool> run_{true};
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

    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    reader.set_elevator_encoder(make_encoder(4));
    reader.set_elevator_absolute_pwm(make_unique<frc::DigitalInput>(4));
    reader.set_elevator_potentiometer(make_unique<frc::AnalogInput>(4));

    reader.set_wrist_encoder(make_encoder(5));
    reader.set_wrist_absolute_pwm(make_unique<frc::DigitalInput>(5));
    reader.set_wrist_potentiometer(make_unique<frc::AnalogInput>(5));

    reader.set_intake_encoder(make_encoder(2));
    reader.set_intake_absolute_pwm(make_unique<frc::DigitalInput>(2));

    reader.set_stilts_encoder(make_encoder(3));
    reader.set_stilts_absolute_pwm(make_unique<frc::DigitalInput>(3));
    reader.set_stilts_potentiometer(make_unique<frc::AnalogInput>(3));

    reader.set_pwm_trigger(make_unique<frc::DigitalInput>(25));
    reader.set_vacuum_sensor(7);

    ::std::thread reader_thread(::std::ref(reader));

    auto imu_trigger = make_unique<frc::DigitalInput>(0);
    ::frc971::wpilib::ADIS16448 imu(frc::SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.SetDummySPI(frc::SPI::Port::kOnboardCS2);
    auto imu_reset = make_unique<frc::DigitalOutput>(1);
    imu.set_reset(imu_reset.get());
    ::std::thread imu_thread(::std::ref(imu));

    // While as of 2/9/18 the drivetrain Victors are SPX, it appears as though
    // they are identical, as far as DrivetrainWriter is concerned, to the SP
    // variety so all the Victors are written as SPs.

    ::frc971::wpilib::DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), false);
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    SuperstructureWriter superstructure_writer;
    superstructure_writer.set_elevator_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    // TODO(austin): Do the vacuum
    superstructure_writer.set_suction_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    superstructure_writer.set_intake_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    superstructure_writer.set_wrist_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    superstructure_writer.set_stilts_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)));

    ::std::thread superstructure_writer_thread(
        ::std::ref(superstructure_writer));

    SolenoidWriter solenoid_writer;
    solenoid_writer.set_intake_roller_talon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonSRX>(10));
    solenoid_writer.set_big_suction_cup(0, 1);
    solenoid_writer.set_small_suction_cup(2, 3);

    ::std::thread solenoid_writer_thread(::std::ref(solenoid_writer));

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

    solenoid_writer.Quit();
    solenoid_writer_thread.join();
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
    superstructure_writer.Quit();
    superstructure_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2019

AOS_ROBOT_CLASS(::y2019::wpilib::WPILibRobot);
