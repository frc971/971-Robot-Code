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
#include "y2022/constants.h"
#include "y2022/control_loops/superstructure/superstructure_can_position_generated.h"
#include "y2022/control_loops/superstructure/superstructure_output_generated.h"
#include "y2022/control_loops/superstructure/superstructure_position_generated.h"

using ::aos::monotonic_clock;
using ::y2022::constants::Values;
namespace superstructure = ::y2022::control_loops::superstructure;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2022 {
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

double climber_pot_translate(double voltage) {
  return voltage * Values::kClimberPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) *
         Values::kClimberPotMetersPerRevolution();
}

double flipper_arms_pot_translate(double voltage) {
  return voltage * Values::kFlipperArmsPotRatio() *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakePotRatio() * (3.0 /*turns*/ / 5.0 /*volts*/) *
         (2 * M_PI /*radians*/);
}

double turret_pot_translate(double voltage) {
  return voltage * Values::kTurretPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    std::max({Values::kMaxDrivetrainEncoderPulsesPerSecond(),
              Values::kMaxIntakeEncoderPulsesPerSecond()});
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    Values::kMaxTurretEncoderPulsesPerSecond();

static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

double catapult_pot_translate(double voltage) {
  return voltage * Values::kCatapultPotRatio() *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

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

  void set_catapult_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    catapult_encoder_.set_encoder(::std::move(encoder));
  }

  void set_catapult_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    catapult_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_catapult_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    catapult_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_heading_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_heading_input_ = ::std::move(sensor);
    imu_heading_reader_.set_input(imu_heading_input_.get());
  }

  void set_yaw_rate_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_yaw_rate_input_ = ::std::move(sensor);
    imu_yaw_rate_reader_.set_input(imu_yaw_rate_input_.get());
  }

  void RunIteration() override {
    {
      auto builder = superstructure_position_sender_.MakeBuilder();

      frc971::PotAndAbsolutePositionT catapult;
      CopyPosition(catapult_encoder_, &catapult,
                   Values::kCatapultEncoderCountsPerRevolution(),
                   Values::kCatapultEncoderRatio(), catapult_pot_translate,
                   false, values_->catapult.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> catapult_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &catapult);

      frc971::RelativePositionT climber;
      CopyPosition(*climber_potentiometer_, &climber, climber_pot_translate,
                   false, values_->climber.potentiometer_offset);
      flatbuffers::Offset<frc971::RelativePosition> climber_offset =
          frc971::RelativePosition::Pack(*builder.fbb(), &climber);

      frc971::RelativePositionT flipper_arm_left;
      CopyPosition(*flipper_arm_left_potentiometer_, &flipper_arm_left,
                   flipper_arms_pot_translate, false,
                   values_->flipper_arm_left.potentiometer_offset);

      frc971::RelativePositionT flipper_arm_right;
      CopyPosition(*flipper_arm_right_potentiometer_, &flipper_arm_right,
                   flipper_arms_pot_translate, true,
                   values_->flipper_arm_right.potentiometer_offset);

      // Intake
      frc971::PotAndAbsolutePositionT intake_front;
      CopyPosition(intake_encoder_front_, &intake_front,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), intake_pot_translate, true,
                   values_->intake_front.potentiometer_offset);
      frc971::PotAndAbsolutePositionT intake_back;
      CopyPosition(intake_encoder_back_, &intake_back,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), intake_pot_translate, true,
                   values_->intake_back.potentiometer_offset);
      frc971::PotAndAbsolutePositionT turret;
      CopyPosition(turret_encoder_, &turret,
                   Values::kTurretEncoderCountsPerRevolution(),
                   Values::kTurretEncoderRatio(), turret_pot_translate, false,
                   values_->turret.potentiometer_offset);

      flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_offset_front =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &intake_front);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_offset_back =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &intake_back);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> turret_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &turret);
      flatbuffers::Offset<frc971::RelativePosition> flipper_arm_left_offset =
          frc971::RelativePosition::Pack(*builder.fbb(), &flipper_arm_left);
      flatbuffers::Offset<frc971::RelativePosition> flipper_arm_right_offset =
          frc971::RelativePosition::Pack(*builder.fbb(), &flipper_arm_right);

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();
      position_builder.add_climber(climber_offset);
      position_builder.add_flipper_arm_left(flipper_arm_left_offset);
      position_builder.add_flipper_arm_right(flipper_arm_right_offset);
      position_builder.add_intake_front(intake_offset_front);
      position_builder.add_intake_back(intake_offset_back);
      position_builder.add_turret(turret_offset);
      position_builder.add_intake_beambreak_front(
          intake_beambreak_front_->Get());
      position_builder.add_intake_beambreak_back(intake_beambreak_back_->Get());
      position_builder.add_turret_beambreak(turret_beambreak_->Get());
      position_builder.add_catapult(catapult_offset);
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
      constexpr double kMaxVelocity = 2000;  // degrees / second
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

  void set_climber_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    climber_potentiometer_ = ::std::move(potentiometer);
  }

  void set_flipper_arm_left_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    flipper_arm_left_potentiometer_ = ::std::move(potentiometer);
  }

  void set_flipper_arm_right_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    flipper_arm_right_potentiometer_ = ::std::move(potentiometer);
  }

  void set_intake_encoder_front(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    intake_encoder_front_.set_encoder(::std::move(encoder));
  }

  void set_intake_encoder_back(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    intake_encoder_back_.set_encoder(::std::move(encoder));
  }

  void set_intake_front_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    intake_encoder_front_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_intake_front_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    intake_encoder_front_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_back_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    intake_encoder_back_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_intake_back_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    intake_encoder_back_.set_potentiometer(::std::move(potentiometer));
  }

  void set_turret_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    turret_encoder_.set_encoder(::std::move(encoder));
  }

  void set_turret_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    turret_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_turret_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    turret_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_beambreak_front(::std::unique_ptr<frc::DigitalInput> sensor) {
    intake_beambreak_front_ = ::std::move(sensor);
  }
  void set_intake_beambreak_back(::std::unique_ptr<frc::DigitalInput> sensor) {
    intake_beambreak_back_ = ::std::move(sensor);
  }
  void set_turret_beambreak(::std::unique_ptr<frc::DigitalInput> sensor) {
    turret_beambreak_ = ::std::move(sensor);
  }

 private:
  std::shared_ptr<const Values> values_;

  aos::Sender<frc971::autonomous::AutonomousMode> auto_mode_sender_;
  aos::Sender<superstructure::Position> superstructure_position_sender_;
  aos::Sender<frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;

  std::array<std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  std::unique_ptr<frc::DigitalInput> intake_beambreak_front_,
      intake_beambreak_back_, turret_beambreak_, imu_heading_input_,
      imu_yaw_rate_input_;

  std::unique_ptr<frc::AnalogInput> climber_potentiometer_,
      flipper_arm_right_potentiometer_, flipper_arm_left_potentiometer_;
  frc971::wpilib::AbsoluteEncoderAndPotentiometer intake_encoder_front_,
      intake_encoder_back_, turret_encoder_, catapult_encoder_;

  frc971::wpilib::DMAPulseWidthReader imu_heading_reader_, imu_yaw_rate_reader_;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(aos::EventLoop *event_loop)
      : frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  void set_climber_falcon(std::unique_ptr<frc::TalonFX> t) {
    climber_falcon_ = std::move(t);
  }

  void set_turret_falcon(::std::unique_ptr<::frc::TalonFX> t) {
    turret_falcon_ = ::std::move(t);
  }

  void set_catapult_falcon_1(::std::unique_ptr<::frc::TalonFX> t) {
    catapult_falcon_1_ = ::std::move(t);
  }

  void set_intake_falcon_front(::std::unique_ptr<frc::TalonFX> t) {
    intake_falcon_front_ = ::std::move(t);
  }

  void set_intake_falcon_back(::std::unique_ptr<frc::TalonFX> t) {
    intake_falcon_back_ = ::std::move(t);
  }

  void set_roller_falcon_front(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    roller_falcon_front_ = ::std::move(t);
    roller_falcon_front_->ConfigSupplyCurrentLimit(
        {true, Values::kIntakeRollerSupplyCurrentLimit(),
         Values::kIntakeRollerSupplyCurrentLimit(), 0});
    roller_falcon_front_->ConfigStatorCurrentLimit(
        {true, Values::kIntakeRollerStatorCurrentLimit(),
         Values::kIntakeRollerStatorCurrentLimit(), 0});
  }

  void set_roller_falcon_back(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    roller_falcon_back_ = ::std::move(t);
    roller_falcon_back_->ConfigSupplyCurrentLimit(
        {true, Values::kIntakeRollerSupplyCurrentLimit(),
         Values::kIntakeRollerSupplyCurrentLimit(), 0});
    roller_falcon_back_->ConfigStatorCurrentLimit(
        {true, Values::kIntakeRollerStatorCurrentLimit(),
         Values::kIntakeRollerStatorCurrentLimit(), 0});
  }

  void set_flipper_arms_falcon(
      ::std::shared_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    flipper_arms_falcon_ = t;
    flipper_arms_falcon_->ConfigSupplyCurrentLimit(
        {true, Values::kFlipperArmSupplyCurrentLimit(),
         Values::kFlipperArmSupplyCurrentLimit(), 0});
    flipper_arms_falcon_->ConfigStatorCurrentLimit(
        {true, Values::kFlipperArmStatorCurrentLimit(),
         Values::kFlipperArmStatorCurrentLimit(), 0});
  }

  ::std::shared_ptr<::ctre::phoenix::motorcontrol::can::TalonFX>
  flipper_arms_falcon() {
    return flipper_arms_falcon_;
  }

  void set_transfer_roller_victor_front(::std::unique_ptr<::frc::VictorSP> t) {
    transfer_roller_victor_front_ = ::std::move(t);
  }

  void set_transfer_roller_victor_back(::std::unique_ptr<::frc::VictorSP> t) {
    transfer_roller_victor_back_ = ::std::move(t);
  }

 private:
  void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    climber_falcon_->SetDisabled();
    roller_falcon_front_->Set(
        ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
    roller_falcon_back_->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled,
                             0);
    flipper_arms_falcon_->Set(
        ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
    intake_falcon_front_->SetDisabled();
    intake_falcon_back_->SetDisabled();
    transfer_roller_victor_front_->SetDisabled();
    transfer_roller_victor_back_->SetDisabled();
    catapult_falcon_1_->SetDisabled();
    turret_falcon_->SetDisabled();
  }

  void Write(const superstructure::Output &output) override {
    WritePwm(output.climber_voltage(), climber_falcon_.get());

    WritePwm(output.intake_voltage_front(), intake_falcon_front_.get());
    WritePwm(output.intake_voltage_back(), intake_falcon_back_.get());
    WriteCan(output.roller_voltage_front(), roller_falcon_front_.get());
    WriteCan(output.roller_voltage_back(), roller_falcon_back_.get());
    WritePwm(output.transfer_roller_voltage_front(),
             transfer_roller_victor_front_.get());
    WritePwm(output.transfer_roller_voltage_back(),
             transfer_roller_victor_back_.get());

    WriteCan(-output.flipper_arms_voltage(), flipper_arms_falcon_.get());

    WritePwm(output.catapult_voltage(), catapult_falcon_1_.get());

    WritePwm(-output.turret_voltage(), turret_falcon_.get());
  }

  static void WriteCan(const double voltage,
                       ::ctre::phoenix::motorcontrol::can::TalonFX *falcon) {
    falcon->Set(
        ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
        std::clamp(voltage, -kMaxBringupPower, kMaxBringupPower) / 12.0);
  }

  template <typename T>
  static void WritePwm(const double voltage, T *motor) {
    motor->SetSpeed(std::clamp(voltage, -kMaxBringupPower, kMaxBringupPower) /
                    12.0);
  }

  ::std::unique_ptr<frc::TalonFX> intake_falcon_front_, intake_falcon_back_;

  ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX>
      roller_falcon_front_, roller_falcon_back_;

  ::std::shared_ptr<::ctre::phoenix::motorcontrol::can::TalonFX>
      flipper_arms_falcon_;

  ::std::unique_ptr<::frc::TalonFX> turret_falcon_, catapult_falcon_1_,
      climber_falcon_;
  ::std::unique_ptr<::frc::VictorSP> transfer_roller_victor_front_,
      transfer_roller_victor_back_;
};

class CANSensorReader {
 public:
  CANSensorReader(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        can_position_sender_(
            event_loop->MakeSender<superstructure::CANPosition>(
                "/superstructure")) {
    event_loop->SetRuntimeRealtimePriority(16);

    phased_loop_handler_ =
        event_loop_->AddPhasedLoop([this](int) { Loop(); }, kPeriod);
    phased_loop_handler_->set_name("CAN SensorReader Loop");

    event_loop->OnRun([this]() { Loop(); });
  }

  void set_flipper_arms_falcon(
      ::std::shared_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    flipper_arms_falcon_ = std::move(t);
  }

 private:
  void Loop() {
    auto builder = can_position_sender_.MakeBuilder();
    superstructure::CANPosition::Builder can_position_builder =
        builder.MakeBuilder<superstructure::CANPosition>();
    can_position_builder.add_flipper_arm_integrated_sensor_velocity(
        flipper_arms_falcon_->GetSelectedSensorVelocity() *
        kVelocityConversion);
    builder.CheckOk(builder.Send(can_position_builder.Finish()));
  }

  static constexpr std::chrono::milliseconds kPeriod =
      std::chrono::milliseconds(20);
  // 2048 encoder counts / 100 ms to rad/sec
  static constexpr double kVelocityConversion = (2.0 * M_PI / 2048) * 0.100;
  aos::EventLoop *event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handler_;

  ::std::shared_ptr<::ctre::phoenix::motorcontrol::can::TalonFX>
      flipper_arms_falcon_;
  aos::Sender<superstructure::CANPosition> can_position_sender_;
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
    sensor_reader.set_drivetrain_left_encoder(make_encoder(1));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(0));

    sensor_reader.set_intake_encoder_front(make_encoder(3));
    sensor_reader.set_intake_front_absolute_pwm(
        make_unique<frc::DigitalInput>(3));
    sensor_reader.set_intake_front_potentiometer(
        make_unique<frc::AnalogInput>(3));

    sensor_reader.set_intake_encoder_back(make_encoder(4));
    sensor_reader.set_intake_back_absolute_pwm(
        make_unique<frc::DigitalInput>(4));
    sensor_reader.set_intake_back_potentiometer(
        make_unique<frc::AnalogInput>(4));

    sensor_reader.set_turret_encoder(make_encoder(5));
    sensor_reader.set_turret_absolute_pwm(make_unique<frc::DigitalInput>(5));
    sensor_reader.set_turret_potentiometer(make_unique<frc::AnalogInput>(5));

    // TODO(milind): correct intake beambreak ports once set
    sensor_reader.set_intake_beambreak_front(make_unique<frc::DigitalInput>(1));
    sensor_reader.set_intake_beambreak_back(make_unique<frc::DigitalInput>(6));
    sensor_reader.set_turret_beambreak(make_unique<frc::DigitalInput>(7));

    sensor_reader.set_climber_potentiometer(make_unique<frc::AnalogInput>(7));

    sensor_reader.set_flipper_arm_left_potentiometer(
        make_unique<frc::AnalogInput>(0));
    sensor_reader.set_flipper_arm_right_potentiometer(
        make_unique<frc::AnalogInput>(1));

    // TODO(milind): correct catapult encoder and absolute pwm ports
    sensor_reader.set_catapult_encoder(make_encoder(2));
    sensor_reader.set_catapult_absolute_pwm(
        std::make_unique<frc::DigitalInput>(2));
    sensor_reader.set_catapult_potentiometer(
        std::make_unique<frc::AnalogInput>(2));

    sensor_reader.set_heading_input(make_unique<frc::DigitalInput>(9));
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(8));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), false);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), true);

    SuperstructureWriter superstructure_writer(&output_event_loop);

    superstructure_writer.set_turret_falcon(make_unique<::frc::TalonFX>(3));
    superstructure_writer.set_roller_falcon_front(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonFX>(0));
    superstructure_writer.set_roller_falcon_back(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonFX>(1));

    superstructure_writer.set_transfer_roller_victor_front(
        make_unique<::frc::VictorSP>(6));
    superstructure_writer.set_transfer_roller_victor_back(
        make_unique<::frc::VictorSP>(5));

    superstructure_writer.set_intake_falcon_front(make_unique<frc::TalonFX>(2));
    superstructure_writer.set_intake_falcon_back(make_unique<frc::TalonFX>(4));
    superstructure_writer.set_climber_falcon(make_unique<frc::TalonFX>(8));
    superstructure_writer.set_flipper_arms_falcon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonFX>(2));

    superstructure_writer.set_catapult_falcon_1(make_unique<::frc::TalonFX>(9));

    AddLoop(&output_event_loop);

    // Thread 5
    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    CANSensorReader can_sensor_reader(&can_sensor_reader_event_loop);
    can_sensor_reader.set_flipper_arms_falcon(
        superstructure_writer.flipper_arms_falcon());
    AddLoop(&can_sensor_reader_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2022

AOS_ROBOT_CLASS(::y2022::wpilib::WPILibRobot);
