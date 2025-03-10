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

#include "ctre/phoenix6/TalonFX.hpp"

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
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/input/robot_state_generated.h"
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
#include "y2021_bot3/constants.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_output_generated.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_position_generated.h"

using ::aos::monotonic_clock;
using ::y2021_bot3::constants::Values;
namespace superstructure = ::y2021_bot3::control_loops::superstructure;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2021_bot3::wpilib {
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

constexpr double kMaxFastEncoderPulsesPerSecond =
    Values::kMaxDrivetrainEncoderPulsesPerSecond();
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    kMaxFastEncoderPulsesPerSecond;

static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

void PrintConfigs(ctre::phoenix6::hardware::TalonFX *talon) {
  ctre::phoenix6::configs::TalonFXConfiguration configuration;
  ctre::phoenix::StatusCode status =
      talon->GetConfigurator().Refresh(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to get falcon configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }
  AOS_LOG(INFO, "configuration: %s", configuration.ToString().c_str());
}

void WriteConfigs(ctre::phoenix6::hardware::TalonFX *talon,
                  double stator_current_limit, double supply_current_limit) {
  ctre::phoenix6::configs::CurrentLimitsConfigs current_limits;
  current_limits.StatorCurrentLimit =
      units::current::ampere_t{stator_current_limit};
  current_limits.StatorCurrentLimitEnable = true;
  current_limits.SupplyCurrentLimit =
      units::current::ampere_t{supply_current_limit};
  current_limits.SupplyCurrentLimitEnable = true;

  ctre::phoenix6::configs::TalonFXConfiguration configuration;
  configuration.CurrentLimits = current_limits;

  ctre::phoenix::StatusCode status =
      talon->GetConfigurator().Apply(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to set falcon configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }

  PrintConfigs(talon);
}

void Disable(ctre::phoenix6::hardware::TalonFX *talon) {
  ctre::phoenix6::controls::DutyCycleOut stop_command(0.0);
  stop_command.UpdateFreqHz = 0_Hz;
  stop_command.EnableFOC = true;

  talon->SetControl(stop_command);
}

}  // namespace

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        auto_mode_sender_(
            event_loop->MakeSender<::frc971::autonomous::AutonomousMode>(
                "/autonomous")),
        superstructure_position_sender_(
            event_loop->MakeSender<superstructure::Position>(
                "/superstructure")),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                    "/drivetrain")) {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    UpdateMediumEncoderFilterHz(kMaxMediumEncoderPulsesPerSecond);
  }

  // Auto mode switches.
  void set_autonomous_mode(int i, ::std::unique_ptr<frc::DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void RunIteration() override {
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();
      frc971::control_loops::drivetrain::Position::Builder drivetrain_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();
      drivetrain_builder.add_left_encoder(
          drivetrain_translate(drivetrain_left_encoder_->GetRaw()));
      drivetrain_builder.add_left_speed(
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));

      drivetrain_builder.add_right_encoder(
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw()));
      drivetrain_builder.add_right_speed(-drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod()));

      builder.CheckOk(builder.Send(drivetrain_builder.Finish()));
    }

    {
      auto builder = superstructure_position_sender_.MakeBuilder();
      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();
      builder.CheckOk(builder.Send(position_builder.Finish()));
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
  ::aos::Sender<::frc971::autonomous::AutonomousMode> auto_mode_sender_;
  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;

  ::std::array<::std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  void set_intake_falcon(
      ::std::unique_ptr<::ctre::phoenix6::hardware::TalonFX> t) {
    intake_falcon_ = ::std::move(t);
    ConfigureRollerFalcon(intake_falcon_.get());
  }

  void set_outtake_falcon(
      ::std::unique_ptr<::ctre::phoenix6::hardware::TalonFX> t) {
    outtake_falcon_ = ::std::move(t);
    ConfigureRollerFalcon(outtake_falcon_.get());
  }

  void set_climber_falcon(
      ::std::unique_ptr<::ctre::phoenix6::hardware::TalonFX> t) {
    climber_falcon_ = ::std::move(t);
    ctre::phoenix6::configs::CurrentLimitsConfigs current_limits;
    current_limits.SupplyCurrentLimit =
        units::current::ampere_t{Values::kClimberSupplyCurrentLimit()};
    current_limits.SupplyCurrentLimitEnable = true;

    ctre::phoenix6::configs::TalonFXConfiguration configuration;
    configuration.CurrentLimits = current_limits;

    ctre::phoenix::StatusCode status =
        climber_falcon_->GetConfigurator().Apply(configuration);
    if (!status.IsOK()) {
      AOS_LOG(ERROR, "Failed to set falcon configuration: %s: %s",
              status.GetName(), status.GetDescription());
    }

    PrintConfigs(climber_falcon_.get());
  }

 private:
  void ConfigureRollerFalcon(::ctre::phoenix6::hardware::TalonFX *falcon) {
    WriteConfigs(falcon, Values::kRollerSupplyCurrentLimit(),
                 Values::kRollerStatorCurrentLimit());
  }

  void WriteToFalcon(const double voltage,
                     ::ctre::phoenix6::hardware::TalonFX *falcon) {
    ctre::phoenix6::controls::DutyCycleOut control(
        std::clamp(voltage, -kMaxBringupPower, kMaxBringupPower) / 12.0);
    control.UpdateFreqHz = 0_Hz;
    control.EnableFOC = true;

    falcon->SetControl(control);
  }

  void Write(const superstructure::Output &output) override {
    WriteToFalcon(output.intake_volts(), intake_falcon_.get());
    WriteToFalcon(output.outtake_volts(), outtake_falcon_.get());

    WriteToFalcon(-output.climber_volts(), climber_falcon_.get());
  }

  void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    Disable(climber_falcon_.get());
    Disable(intake_falcon_.get());
    Disable(outtake_falcon_.get());
  }

  ::std::unique_ptr<::ctre::phoenix6::hardware::TalonFX> intake_falcon_,
      outtake_falcon_;

  ::std::unique_ptr<::ctre::phoenix6::hardware::TalonFX> climber_falcon_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run() override {
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
    SensorReader sensor_reader(&sensor_reader_event_loop);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(0));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(1));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), false);

    SuperstructureWriter superstructure_writer(&output_event_loop);
    superstructure_writer.set_intake_falcon(
        make_unique<::ctre::phoenix6::hardware::TalonFX>(0));
    superstructure_writer.set_outtake_falcon(
        make_unique<::ctre::phoenix6::hardware::TalonFX>(1));
    superstructure_writer.set_climber_falcon(
        make_unique<::ctre::phoenix6::hardware::TalonFX>(2));

    AddLoop(&output_event_loop);

    RunLoops();
  }
};

}  // namespace y2021_bot3::wpilib

AOS_ROBOT_CLASS(::y2021_bot3::wpilib::WPILibRobot);
