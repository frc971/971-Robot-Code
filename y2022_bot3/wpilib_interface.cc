#include <unistd.h>

#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "ctre/phoenix/CANifier.h"
#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/Servo.h"
#include "frc971/wpilib/ahal/TalonFX.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2022_bot3/constants.h"
#include "y2022_bot3/control_loops/superstructure/superstructure_output_generated.h"
#include "y2022_bot3/control_loops/superstructure/superstructure_position_generated.h"

using ::aos::monotonic_clock;
using ::y2022_bot3::constants::Values;
namespace superstructure = ::y2022_bot3::control_loops::superstructure;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2022_bot3 {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakePotRatio() * (3.0 /*turns*/ / 5.0 /*volts*/) *
         (2 * M_PI /*radians*/);
}

double climber_pot_translate(double voltage) {
  return voltage * Values::kClimberPotRatio() *
         (3.0 /*turns*/ / 5.0 /*volts*/) *
         Values::kClimberPotMetersPerRevolution();
}

// TODO(niko): Might have to move these to medium once we know the actual values
constexpr double kMaxFastEncoderPulsesPerSecond = std::max(
    {Values::kMaxDrivetrainEncoderPulsesPerSecond(),
     Values::kMaxIntakeEncoderPulsesPerSecond(),
     Values::kMaxClimberEncoderPulsesPerSecond()});
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond = 0;

static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

}  // namespace

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop,
               std::shared_ptr<const Values> values)
      : ::frc971::wpilib::SensorReader(event_loop),
        values_(std::move(values)),
        auto_mode_sender_(
            event_loop->MakeSender<::frc971::autonomous::AutonomousMode>(
                "/autonomous")),
        superstructure_position_sender_(
            event_loop->MakeSender<superstructure::Position>(
                "/superstructure")),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                    "/drivetrain")),
        gyro_sender_(event_loop->MakeSender<::frc971::sensors::GyroReading>(
            "/drivetrain")) {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    UpdateMediumEncoderFilterHz(kMaxMediumEncoderPulsesPerSecond);
  }

  void Start() override {
    // TODO(Ravago): Figure out why adding multiple DMA readers results in weird
    // behavior
    // AddToDMA(&imu_heading_reader_);
    AddToDMA(&imu_yaw_rate_reader_);
  }

  // Auto mode switches.
  void set_autonomous_mode(int i, ::std::unique_ptr<frc::DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void set_heading_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_heading_input_ = ::std::move(sensor);
    imu_heading_reader_.set_input(imu_heading_input_.get());
  }

  void set_yaw_rate_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_yaw_rate_input_ = ::std::move(sensor);
    imu_yaw_rate_reader_.set_input(imu_yaw_rate_input_.get());
  }

  void set_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_intake_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_climber_encoder_right(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    climber_encoder_right_.set_encoder(::std::move(encoder));
  }

  void set_climber_right_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    climber_encoder_right_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_climber_right_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    climber_encoder_right_.set_potentiometer(::std::move(potentiometer));
  }

  void set_climber_encoder_left(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    climber_encoder_left_.set_encoder(::std::move(encoder));
  }

  void set_climber_left_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    climber_encoder_left_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_climber_left_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    climber_encoder_left_.set_potentiometer(::std::move(potentiometer));
  }

  void RunIteration() override {
    {
      auto builder = superstructure_position_sender_.MakeBuilder();

      // Climbers
      frc971::PotAndAbsolutePositionT climber_right;
      CopyPosition(climber_encoder_right_, &climber_right,
                   Values::kClimberEncoderCountsPerRevolution(),
                   (Values::kClimberEncoderRatio() *
                     Values::kClimberEncoderCountsPerRevolution()) /
                    (2.0 * M_PI),
                   climber_pot_translate, true,
                   values_->climber_right.potentiometer_offset);

      frc971::PotAndAbsolutePositionT climber_left;
      CopyPosition(climber_encoder_left_, &climber_left,
                   Values::kClimberEncoderCountsPerRevolution(),
                   (Values::kClimberEncoderRatio() *
                     Values::kClimberEncoderCountsPerRevolution()) /
                    (2.0 * M_PI),
                   climber_pot_translate, true,
                   values_->climber_left.potentiometer_offset);

      // Intake
      frc971::PotAndAbsolutePositionT intake;
      CopyPosition(intake_encoder_, &intake,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), intake_pot_translate, true,
                   values_->intake.potentiometer_offset);

      flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &intake);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> climber_offset_right =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &climber_right);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> climber_offset_left =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &climber_left);

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();
      position_builder.add_intake(intake_offset);
      position_builder.add_climber_right(climber_offset_right);
      position_builder.add_climber_left(climber_offset_left);
      builder.CheckOk(builder.Send(position_builder.Finish()));
    }

    {
      auto builder = drivetrain_position_sender_.MakeBuilder();
      frc971::control_loops::drivetrain::Position::Builder drivetrain_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();
      drivetrain_builder.add_left_encoder(
          constants::Values::DrivetrainEncoderToMeters(
              drivetrain_left_encoder_->GetRaw()));
      drivetrain_builder.add_left_speed(
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));

      drivetrain_builder.add_right_encoder(
          -constants::Values::DrivetrainEncoderToMeters(
              drivetrain_right_encoder_->GetRaw()));
      drivetrain_builder.add_right_speed(-drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod()));

      builder.CheckOk(builder.Send(drivetrain_builder.Finish()));
    }

    {
      auto builder = gyro_sender_.MakeBuilder();
      ::frc971::sensors::GyroReading::Builder gyro_reading_builder =
          builder.MakeBuilder<::frc971::sensors::GyroReading>();
      // +/- 2000 deg / sec
      constexpr double kMaxVelocity = 4000;  // degrees / second
      constexpr double kVelocityRadiansPerSecond =
          kMaxVelocity / 360 * (2.0 * M_PI);

      // Only part of the full range is used to prevent being 100% on or off.
      constexpr double kScaledRangeLow = 0.1;
      constexpr double kScaledRangeHigh = 0.9;

      constexpr double kPWMFrequencyHz = 200;
      double heading_duty_cycle =
          imu_heading_reader_.last_width() * kPWMFrequencyHz;
      double velocity_duty_cycle =
          imu_yaw_rate_reader_.last_width() * kPWMFrequencyHz;

      constexpr double kDutyCycleScale =
          1 / (kScaledRangeHigh - kScaledRangeLow);
      // scale from 0.1 - 0.9 to 0 - 1
      double rescaled_heading_duty_cycle =
          (heading_duty_cycle - kScaledRangeLow) * kDutyCycleScale;
      double rescaled_velocity_duty_cycle =
          (velocity_duty_cycle - kScaledRangeLow) * kDutyCycleScale;

      if (!std::isnan(rescaled_heading_duty_cycle)) {
        gyro_reading_builder.add_angle(rescaled_heading_duty_cycle *
                                       (2.0 * M_PI));
      }
      if (!std::isnan(rescaled_velocity_duty_cycle)) {
        gyro_reading_builder.add_velocity((rescaled_velocity_duty_cycle - 0.5) *
                                          kVelocityRadiansPerSecond);
      }
      builder.CheckOk(builder.Send(gyro_reading_builder.Finish()));
    }

    {
      auto builder = auto_mode_sender_.MakeBuilder();

      uint32_t mode = 0;
      for (size_t i = 0; i < autonomous_modes_.size(); ++i) {
        if (autonomous_modes_[i] && autonomous_modes_[i]->Get()) {
          mode |= 1 << i;
        }
      }

      auto auto_mode_builder =
          builder.MakeBuilder<frc971::autonomous::AutonomousMode>();

      auto_mode_builder.add_mode(mode);

      builder.CheckOk(builder.Send(auto_mode_builder.Finish()));
    }
  }

 private:
  std::shared_ptr<const Values> values_;

  aos::Sender<frc971::autonomous::AutonomousMode> auto_mode_sender_;
  aos::Sender<superstructure::Position> superstructure_position_sender_;
  aos::Sender<frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;

  std::array<std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  std::unique_ptr<frc::DigitalInput> imu_heading_input_, imu_yaw_rate_input_;

  frc971::wpilib::DMAPulseWidthReader imu_heading_reader_, imu_yaw_rate_reader_;

  frc971::wpilib::AbsoluteEncoderAndPotentiometer intake_encoder_,
      climber_encoder_left_, climber_encoder_right_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run() override {
    std::shared_ptr<const Values> values =
        std::make_shared<const Values>(constants::MakeValues());

    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("aos_config.json");

    // Thread 1.
    ::aos::ShmEventLoop joystick_sender_event_loop(&config.message());
    ::frc971::wpilib::JoystickSender joystick_sender(
        &joystick_sender_event_loop);
    AddLoop(&joystick_sender_event_loop);

    // Thread 2.
    ::aos::ShmEventLoop pdp_fetcher_event_loop(&config.message());
    ::frc971::wpilib::PDPFetcher pdp_fetcher(&pdp_fetcher_event_loop);
    AddLoop(&pdp_fetcher_event_loop);

    // Thread 3.
    ::aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    SensorReader sensor_reader(&sensor_reader_event_loop, values);
    sensor_reader.set_pwm_trigger(true);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(0));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(1));

    sensor_reader.set_heading_input(make_unique<frc::DigitalInput>(8));
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(9));

    sensor_reader.set_intake_encoder(make_encoder(2));
    sensor_reader.set_climber_encoder_right(make_encoder(3));
    sensor_reader.set_climber_encoder_left(make_encoder(4));

    sensor_reader.set_intake_absolute_pwm(make_unique<frc::DigitalInput>(0));
    sensor_reader.set_climber_right_absolute_pwm(
        make_unique<frc::DigitalInput>(1));
    sensor_reader.set_climber_left_absolute_pwm(
        make_unique<frc::DigitalInput>(2));

    sensor_reader.set_intake_potentiometer(make_unique<frc::AnalogInput>(0));
    sensor_reader.set_climber_right_potentiometer(
        make_unique<frc::AnalogInput>(1));
    sensor_reader.set_climber_left_potentiometer(
        make_unique<frc::AnalogInput>(2));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), false);
    drivetrain_writer.set_left_controller1(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), false);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)), true);
    drivetrain_writer.set_right_controller1(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)), true);
    AddLoop(&output_event_loop);

    // Thread 5.
    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2022_bot3

AOS_ROBOT_CLASS(::y2022_bot3::wpilib::WPILibRobot);
