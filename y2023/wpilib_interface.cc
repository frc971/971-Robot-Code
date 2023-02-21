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
#include "aos/containers/sized_array.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "ctre/phoenix/cci/Diagnostics_CCI.h"
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "ctre/phoenixpro/TalonFX.hpp"
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
#include "y2023/constants.h"
#include "y2023/control_loops/drivetrain/drivetrain_can_position_generated.h"
#include "y2023/control_loops/superstructure/superstructure_output_generated.h"
#include "y2023/control_loops/superstructure/superstructure_position_generated.h"

DEFINE_bool(ctre_diag_server, false,
            "If true, enable the diagnostics server for interacting with "
            "devices on the CAN bus using Phoenix Tuner");

using ::aos::monotonic_clock;
using ::y2023::constants::Values;
namespace superstructure = ::y2023::control_loops::superstructure;
namespace drivetrain = ::y2023::control_loops::drivetrain;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2023 {
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

double proximal_pot_translate(double voltage) {
  return voltage * Values::kProximalPotRadiansPerVolt();
}

double distal_pot_translate(double voltage) {
  return voltage * Values::kDistalPotRadiansPerVolt();
}

double roll_joint_pot_translate(double voltage) {
  return voltage * Values::kRollJointPotRadiansPerVolt();
}

constexpr double kMaxFastEncoderPulsesPerSecond = std::max({
    Values::kMaxDrivetrainEncoderPulsesPerSecond(),
    Values::kMaxProximalEncoderPulsesPerSecond(),
    Values::kMaxDistalEncoderPulsesPerSecond(),
    Values::kMaxRollJointEncoderPulsesPerSecond(),
    Values::kMaxWristEncoderPulsesPerSecond(),
});
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

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

  void RunIteration() override {
    superstructure_reading_->Set(true);
    {
      auto builder = superstructure_position_sender_.MakeBuilder();
      frc971::PotAndAbsolutePositionT proximal;
      CopyPosition(proximal_encoder_, &proximal,
                   Values::kProximalEncoderCountsPerRevolution(),
                   Values::kProximalEncoderRatio(), proximal_pot_translate,
                   false, values_->arm_proximal.potentiometer_offset);
      frc971::PotAndAbsolutePositionT distal;
      CopyPosition(distal_encoder_, &distal,
                   Values::kDistalEncoderCountsPerRevolution(),
                   Values::kDistalEncoderRatio(), distal_pot_translate, false,
                   values_->arm_distal.potentiometer_offset);
      frc971::PotAndAbsolutePositionT roll_joint;
      CopyPosition(roll_joint_encoder_, &roll_joint,
                   Values::kRollJointEncoderCountsPerRevolution(),
                   Values::kRollJointEncoderRatio(), roll_joint_pot_translate,
                   false, values_->roll_joint.potentiometer_offset);
      frc971::AbsolutePositionT wrist;
      CopyPosition(wrist_encoder_, &wrist,
                   Values::kWristEncoderCountsPerRevolution(),
                   Values::kWristEncoderRatio(), false);

      flatbuffers::Offset<frc971::PotAndAbsolutePosition> proximal_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &proximal);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> distal_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &distal);
      flatbuffers::Offset<superstructure::ArmPosition> arm_offset =
          superstructure::CreateArmPosition(*builder.fbb(), proximal_offset,
                                            distal_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> roll_joint_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &roll_joint);
      flatbuffers::Offset<frc971::AbsolutePosition> wrist_offset =
          frc971::AbsolutePosition::Pack(*builder.fbb(), &wrist);

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();

      position_builder.add_arm(arm_offset);
      position_builder.add_roll_joint(roll_joint_offset);
      position_builder.add_wrist(wrist_offset);
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

  std::shared_ptr<frc::DigitalOutput> superstructure_reading_;

  void set_superstructure_reading(
      std::shared_ptr<frc::DigitalOutput> superstructure_reading) {
    superstructure_reading_ = superstructure_reading;
  }

  void set_proximal_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    proximal_encoder_.set_encoder(::std::move(encoder));
  }

  void set_proximal_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    proximal_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_proximal_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    proximal_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_distal_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    distal_encoder_.set_encoder(::std::move(encoder));
  }

  void set_distal_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    distal_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_distal_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    distal_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_roll_joint_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    roll_joint_encoder_.set_encoder(::std::move(encoder));
  }

  void set_roll_joint_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    roll_joint_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_roll_joint_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    roll_joint_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_wrist_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
  }

  void set_wrist_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    wrist_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
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

  frc971::wpilib::AbsoluteEncoderAndPotentiometer proximal_encoder_,
      distal_encoder_, roll_joint_encoder_;
  frc971::wpilib::AbsoluteEncoder wrist_encoder_;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(aos::EventLoop *event_loop)
      : frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  std::shared_ptr<frc::DigitalOutput> superstructure_reading_;

  void set_superstructure_reading(
      std::shared_ptr<frc::DigitalOutput> superstructure_reading) {
    superstructure_reading_ = superstructure_reading;
  }

  void set_proximal_falcon(::std::unique_ptr<::frc::TalonFX> t) {
    proximal_falcon_ = ::std::move(t);
  }

  void set_distal_falcon(::std::unique_ptr<::frc::TalonFX> t) {
    distal_falcon_ = ::std::move(t);
  }

  void set_roll_joint_victor(::std::unique_ptr<::frc::VictorSP> t) {
    roll_joint_victor_ = ::std::move(t);
  }

  void set_wrist_victor(::std::unique_ptr<::frc::VictorSP> t) {
    wrist_victor_ = ::std::move(t);
  }

  void set_roller_falcon(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    roller_falcon_ = ::std::move(t);
    roller_falcon_->ConfigSupplyCurrentLimit(
        {true, Values::kRollerSupplyCurrentLimit(),
         Values::kRollerSupplyCurrentLimit(), 0});
    roller_falcon_->ConfigStatorCurrentLimit(
        {true, Values::kRollerStatorCurrentLimit(),
         Values::kRollerStatorCurrentLimit(), 0});
  }

 private:
  void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    proximal_falcon_->SetDisabled();
    distal_falcon_->SetDisabled();
    roll_joint_victor_->SetDisabled();
    wrist_victor_->SetDisabled();
    roller_falcon_->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
  }

  void Write(const superstructure::Output &output) override {
    WritePwm(output.proximal_voltage(), proximal_falcon_.get());
    WritePwm(output.distal_voltage(), distal_falcon_.get());
    WritePwm(output.roll_joint_voltage(), roll_joint_victor_.get());
    WritePwm(output.wrist_voltage(), wrist_victor_.get());
    WriteCan(output.roller_voltage(), roller_falcon_.get());
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

  ::std::unique_ptr<::frc::TalonFX> proximal_falcon_, distal_falcon_;
  ::std::unique_ptr<::frc::VictorSP> roll_joint_victor_, wrist_victor_;
  ::std::shared_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> roller_falcon_;
};

static constexpr int kCANFalconCount = 6;
static constexpr int kCANSignalsCount = 4;
static constexpr units::frequency::hertz_t kCANUpdateFreqHz = 200_Hz;

class Falcon {
 public:
  Falcon(int device_id, std::string canbus,
         aos::SizedArray<ctre::phoenixpro::BaseStatusSignalValue *,
                         kCANFalconCount * kCANSignalsCount> *signals)
      : talon_(device_id, canbus),
        device_id_(device_id),
        device_temp_(talon_.GetDeviceTemp()),
        supply_voltage_(talon_.GetSupplyVoltage()),
        supply_current_(talon_.GetSupplyCurrent()),
        torque_current_(talon_.GetTorqueCurrent()),
        position_(talon_.GetPosition()) {
    // device temp is not timesynced so don't add it to the list of signals
    device_temp_.SetUpdateFrequency(kCANUpdateFreqHz);

    CHECK_EQ(kCANSignalsCount, 4);
    CHECK_NOTNULL(signals);
    CHECK_LE(signals->size() + 4u, signals->capacity());

    supply_voltage_.SetUpdateFrequency(kCANUpdateFreqHz);
    signals->push_back(&supply_voltage_);

    supply_current_.SetUpdateFrequency(kCANUpdateFreqHz);
    signals->push_back(&supply_current_);

    torque_current_.SetUpdateFrequency(kCANUpdateFreqHz);
    signals->push_back(&torque_current_);

    position_.SetUpdateFrequency(kCANUpdateFreqHz);
    signals->push_back(&position_);
  }

  void WriteConfigs(ctre::phoenixpro::signals::InvertedValue invert) {
    inverted_ = invert;

    ctre::phoenixpro::configs::CurrentLimitsConfigs current_limits;
    current_limits.StatorCurrentLimit =
        constants::Values::kDrivetrainStatorCurrentLimit();
    current_limits.StatorCurrentLimitEnable = true;
    current_limits.SupplyCurrentLimit =
        constants::Values::kDrivetrainSupplyCurrentLimit();
    current_limits.SupplyCurrentLimitEnable = true;

    ctre::phoenixpro::configs::MotorOutputConfigs output_configs;
    output_configs.NeutralMode =
        ctre::phoenixpro::signals::NeutralModeValue::Brake;
    output_configs.DutyCycleNeutralDeadband = 0;

    output_configs.Inverted = inverted_;

    ctre::phoenixpro::configs::TalonFXConfiguration configuration;
    configuration.CurrentLimits = current_limits;
    configuration.MotorOutput = output_configs;

    ctre::phoenix::StatusCode status =
        talon_.GetConfigurator().Apply(configuration);
    if (!status.IsOK()) {
      AOS_LOG(ERROR, "Failed to set falcon configuration: %s: %s",
              status.GetName(), status.GetDescription());
    }
  }

  ctre::phoenixpro::hardware::TalonFX *talon() { return &talon_; }

  flatbuffers::Offset<drivetrain::CANFalcon> WritePosition(
      flatbuffers::FlatBufferBuilder *fbb) {
    drivetrain::CANFalcon::Builder builder(*fbb);
    builder.add_id(device_id_);
    builder.add_device_temp(device_temp_.GetValue().value());
    builder.add_supply_voltage(supply_voltage_.GetValue().value());
    builder.add_supply_current(supply_current_.GetValue().value());
    builder.add_torque_current(torque_current_.GetValue().value());

    double invert =
        (inverted_ ==
                 ctre::phoenixpro::signals::InvertedValue::Clockwise_Positive
             ? 1
             : -1);

    builder.add_position(constants::Values::DrivetrainCANEncoderToMeters(
                             position_.GetValue().value()) *
                         invert);

    return builder.Finish();
  }

  // returns the monotonic timestamp of the latest timesynced reading in the
  // timebase of the the syncronized CAN bus clock.
  int64_t GetTimestamp() {
    std::chrono::nanoseconds latest_timestamp =
        torque_current_.GetTimestamp().GetTime();

    return latest_timestamp.count();
  }

  void RefreshNontimesyncedSignals() { device_temp_.Refresh(); };

 private:
  ctre::phoenixpro::hardware::TalonFX talon_;
  int device_id_;

  ctre::phoenixpro::signals::InvertedValue inverted_;

  ctre::phoenixpro::StatusSignalValue<units::temperature::celsius_t>
      device_temp_;
  ctre::phoenixpro::StatusSignalValue<units::voltage::volt_t> supply_voltage_;
  ctre::phoenixpro::StatusSignalValue<units::current::ampere_t> supply_current_,
      torque_current_;
  ctre::phoenixpro::StatusSignalValue<units::angle::turn_t> position_;
};

class CANSensorReader {
 public:
  CANSensorReader(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        can_position_sender_(
            event_loop->MakeSender<drivetrain::CANPosition>("/drivetrain")) {
    event_loop->SetRuntimeRealtimePriority(35);
    timer_handler_ = event_loop->AddTimer([this]() { Loop(); });
    timer_handler_->set_name("CANSensorReader Loop");

    event_loop->OnRun([this]() {
      timer_handler_->Setup(event_loop_->monotonic_now(), 1 / kCANUpdateFreqHz);
    });
  }

  aos::SizedArray<ctre::phoenixpro::BaseStatusSignalValue *,
                  kCANFalconCount * kCANSignalsCount>
      *get_signals_registry() {
    return &signals_;
  };

  void set_falcons(std::shared_ptr<Falcon> right_front,
                   std::shared_ptr<Falcon> right_back,
                   std::shared_ptr<Falcon> right_under,
                   std::shared_ptr<Falcon> left_front,
                   std::shared_ptr<Falcon> left_back,
                   std::shared_ptr<Falcon> left_under) {
    right_front_ = std::move(right_front);
    right_back_ = std::move(right_back);
    right_under_ = std::move(right_under);
    left_front_ = std::move(left_front);
    left_back_ = std::move(left_back);
    left_under_ = std::move(left_under);
  }

 private:
  void Loop() {
    CHECK_EQ(signals_.size(), 24u);
    ctre::phoenix::StatusCode status =
        ctre::phoenixpro::BaseStatusSignalValue::WaitForAll(
            2000_ms, {signals_[0],  signals_[1],  signals_[2],  signals_[3],
                      signals_[4],  signals_[5],  signals_[6],  signals_[7],
                      signals_[8],  signals_[9],  signals_[10], signals_[11],
                      signals_[12], signals_[13], signals_[14], signals_[15],
                      signals_[16], signals_[17], signals_[18], signals_[19],
                      signals_[20], signals_[21], signals_[22], signals_[23]});

    if (!status.IsOK()) {
      AOS_LOG(ERROR, "Failed to read signals from falcons: %s: %s",
              status.GetName(), status.GetDescription());
    }

    auto builder = can_position_sender_.MakeBuilder();

    for (auto falcon : {right_front_, right_back_, right_under_, left_front_,
                        left_back_, left_under_}) {
      falcon->RefreshNontimesyncedSignals();
    }

    aos::SizedArray<flatbuffers::Offset<drivetrain::CANFalcon>, kCANFalconCount>
        falcons;

    for (auto falcon : {right_front_, right_back_, right_under_, left_front_,
                        left_back_, left_under_}) {
      falcons.push_back(falcon->WritePosition(builder.fbb()));
    }

    auto falcons_list =
        builder.fbb()->CreateVector<flatbuffers::Offset<drivetrain::CANFalcon>>(
            falcons);

    drivetrain::CANPosition::Builder can_position_builder =
        builder.MakeBuilder<drivetrain::CANPosition>();

    can_position_builder.add_falcons(falcons_list);
    can_position_builder.add_timestamp(right_front_->GetTimestamp());
    can_position_builder.add_status(static_cast<int>(status));

    builder.CheckOk(builder.Send(can_position_builder.Finish()));
  }

  aos::EventLoop *event_loop_;

  aos::SizedArray<ctre::phoenixpro::BaseStatusSignalValue *,
                  kCANFalconCount * kCANSignalsCount>
      signals_;
  aos::Sender<drivetrain::CANPosition> can_position_sender_;

  std::shared_ptr<Falcon> right_front_, right_back_, right_under_, left_front_,
      left_back_, left_under_;

  // Pointer to the timer handler used to modify the wakeup.
  ::aos::TimerHandler *timer_handler_;
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler<
                             ::frc971::control_loops::drivetrain::Output> {
 public:
  DrivetrainWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<
            ::frc971::control_loops::drivetrain::Output>(event_loop,
                                                         "/drivetrain") {
    event_loop->SetRuntimeRealtimePriority(
        constants::Values::kDrivetrainWriterPriority);

    if (!FLAGS_ctre_diag_server) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
        constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
    ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
        constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");

    event_loop->OnRun([this]() { WriteConfigs(); });
  }

  void set_falcons(std::shared_ptr<Falcon> right_front,
                   std::shared_ptr<Falcon> right_back,
                   std::shared_ptr<Falcon> right_under,
                   std::shared_ptr<Falcon> left_front,
                   std::shared_ptr<Falcon> left_back,
                   std::shared_ptr<Falcon> left_under) {
    right_front_ = std::move(right_front);
    right_back_ = std::move(right_back);
    right_under_ = std::move(right_under);
    left_front_ = std::move(left_front);
    left_back_ = std::move(left_back);
    left_under_ = std::move(left_under);
  }

  void set_right_inverted(ctre::phoenixpro::signals::InvertedValue invert) {
    right_inverted_ = invert;
  }

  void set_left_inverted(ctre::phoenixpro::signals::InvertedValue invert) {
    left_inverted_ = invert;
  }

 private:
  void WriteConfigs() {
    for (auto falcon :
         {right_front_.get(), right_back_.get(), right_under_.get()}) {
      falcon->WriteConfigs(right_inverted_);
    }

    for (auto falcon :
         {left_front_.get(), left_back_.get(), left_under_.get()}) {
      falcon->WriteConfigs(left_inverted_);
    }
  }

  void Write(
      const ::frc971::control_loops::drivetrain::Output &output) override {
    ctre::phoenixpro::controls::DutyCycleOut left_control(
        SafeSpeed(output.left_voltage()));
    left_control.UpdateFreqHz = 0_Hz;
    left_control.EnableFOC = true;

    ctre::phoenixpro::controls::DutyCycleOut right_control(
        SafeSpeed(output.right_voltage()));
    right_control.UpdateFreqHz = 0_Hz;
    right_control.EnableFOC = true;

    for (auto falcon :
         {left_front_.get(), left_back_.get(), left_under_.get()}) {
      ctre::phoenix::StatusCode status =
          falcon->talon()->SetControl(left_control);

      if (!status.IsOK()) {
        AOS_LOG(ERROR, "Failed to write control to falcon: %s: %s",
                status.GetName(), status.GetDescription());
      }
    }

    for (auto falcon :
         {right_front_.get(), right_back_.get(), right_under_.get()}) {
      ctre::phoenix::StatusCode status =
          falcon->talon()->SetControl(right_control);

      if (!status.IsOK()) {
        AOS_LOG(ERROR, "Failed to write control to falcon: %s: %s",
                status.GetName(), status.GetDescription());
      }
    }
  }

  void Stop() override {
    AOS_LOG(WARNING, "drivetrain output too old\n");
    ctre::phoenixpro::controls::NeutralOut stop_command;
    for (auto falcon :
         {right_front_.get(), right_back_.get(), right_under_.get(),
          left_front_.get(), left_back_.get(), left_under_.get()}) {
      falcon->talon()->SetControl(stop_command);
    }
  }

  double SafeSpeed(double voltage) {
    return (::aos::Clip(voltage, -kMaxBringupPower, kMaxBringupPower) / 12.0);
  }

  ctre::phoenixpro::signals::InvertedValue left_inverted_, right_inverted_;
  std::shared_ptr<Falcon> right_front_, right_back_, right_under_, left_front_,
      left_back_, left_under_;
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

    std::shared_ptr<frc::DigitalOutput> superstructure_reading =
        make_unique<frc::DigitalOutput>(25);

    // Thread 3.
    ::aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    SensorReader sensor_reader(&sensor_reader_event_loop, values);
    sensor_reader.set_pwm_trigger(true);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(1));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(0));
    sensor_reader.set_superstructure_reading(superstructure_reading);
    sensor_reader.set_heading_input(make_unique<frc::DigitalInput>(1));
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(0));

    sensor_reader.set_proximal_encoder(make_encoder(3));
    sensor_reader.set_proximal_absolute_pwm(make_unique<frc::DigitalInput>(3));
    sensor_reader.set_proximal_potentiometer(make_unique<frc::AnalogInput>(3));

    sensor_reader.set_distal_encoder(make_encoder(2));
    sensor_reader.set_distal_absolute_pwm(make_unique<frc::DigitalInput>(2));
    sensor_reader.set_distal_potentiometer(make_unique<frc::AnalogInput>(2));

    sensor_reader.set_roll_joint_encoder(make_encoder(5));
    sensor_reader.set_roll_joint_absolute_pwm(
        make_unique<frc::DigitalInput>(5));
    sensor_reader.set_roll_joint_potentiometer(
        make_unique<frc::AnalogInput>(5));

    sensor_reader.set_wrist_encoder(make_encoder(4));
    sensor_reader.set_wrist_absolute_pwm(make_unique<frc::DigitalInput>(4));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    CANSensorReader can_sensor_reader(&can_sensor_reader_event_loop);

    std::shared_ptr<Falcon> right_front = std::make_shared<Falcon>(
        1, "Drivetrain Bus", can_sensor_reader.get_signals_registry());
    std::shared_ptr<Falcon> right_back = std::make_shared<Falcon>(
        2, "Drivetrain Bus", can_sensor_reader.get_signals_registry());
    std::shared_ptr<Falcon> right_under = std::make_shared<Falcon>(
        3, "Drivetrain Bus", can_sensor_reader.get_signals_registry());
    std::shared_ptr<Falcon> left_front = std::make_shared<Falcon>(
        4, "Drivetrain Bus", can_sensor_reader.get_signals_registry());
    std::shared_ptr<Falcon> left_back = std::make_shared<Falcon>(
        5, "Drivetrain Bus", can_sensor_reader.get_signals_registry());
    std::shared_ptr<Falcon> left_under = std::make_shared<Falcon>(
        6, "Drivetrain Bus", can_sensor_reader.get_signals_registry());

    can_sensor_reader.set_falcons(right_front, right_back, right_under,
                                  left_front, left_back, left_under);

    AddLoop(&can_sensor_reader_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    DrivetrainWriter drivetrain_writer(&output_event_loop);

    drivetrain_writer.set_falcons(right_front, right_back, right_under,
                                  left_front, left_back, left_under);
    drivetrain_writer.set_right_inverted(
        ctre::phoenixpro::signals::InvertedValue::Clockwise_Positive);
    drivetrain_writer.set_left_inverted(
        ctre::phoenixpro::signals::InvertedValue::CounterClockwise_Positive);
    SuperstructureWriter superstructure_writer(&output_event_loop);

    superstructure_writer.set_proximal_falcon(make_unique<::frc::TalonFX>(1));
    superstructure_writer.set_distal_falcon(make_unique<::frc::TalonFX>(0));

    superstructure_writer.set_roll_joint_victor(
        make_unique<::frc::VictorSP>(3));
    superstructure_writer.set_wrist_victor(make_unique<::frc::VictorSP>(2));

    superstructure_writer.set_roller_falcon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonFX>(0));

    superstructure_writer.set_superstructure_reading(superstructure_reading);

    AddLoop(&output_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2023

AOS_ROBOT_CLASS(::y2023::wpilib::WPILibRobot);
