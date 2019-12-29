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
#include "ctre/phoenix/CANifier.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/make_unique.h"
#include "aos/realtime.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
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
#include "y2019/camera_log_generated.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/camera_generated.h"
#include "y2019/control_loops/superstructure/superstructure_output_generated.h"
#include "y2019/control_loops/superstructure/superstructure_position_generated.h"
#include "y2019/jevois/spi.h"
#include "y2019/status_light_generated.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::y2019::constants::Values;
using ::aos::monotonic_clock;
namespace superstructure = ::y2019::control_loops::superstructure;
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

  void set_platform_left_detect(
      ::std::unique_ptr<frc::DigitalInput> platform_left_detect) {
    platform_left_detect_ = ::std::move(platform_left_detect);
  }

  void set_platform_right_detect(
      ::std::unique_ptr<frc::DigitalInput> platform_right_detect) {
    platform_right_detect_ = ::std::move(platform_right_detect);
  }

  // Vacuum pressure sensor
  void set_vacuum_sensor(int port) {
    vacuum_sensor_ = make_unique<frc::AnalogInput>(port);
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

      builder.Send(drivetrain_builder.Finish());
    }
    const auto values = constants::GetValues();

    {
      auto builder = superstructure_position_sender_.MakeBuilder();

      // Elevator
      frc971::PotAndAbsolutePositionT elevator;
      CopyPosition(elevator_encoder_, &elevator,
                   Values::kElevatorEncoderCountsPerRevolution(),
                   Values::kElevatorEncoderRatio(), elevator_pot_translate,
                   false, values.elevator.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> elevator_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &elevator);

      // Intake
      frc971::AbsolutePositionT intake_joint;
      CopyPosition(intake_encoder_, &intake_joint,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), false);
      flatbuffers::Offset<frc971::AbsolutePosition> intake_joint_offset =
          frc971::AbsolutePosition::Pack(*builder.fbb(), &intake_joint);

      // Wrist
      frc971::PotAndAbsolutePositionT wrist;
      CopyPosition(wrist_encoder_, &wrist,
                   Values::kWristEncoderCountsPerRevolution(),
                   Values::kWristEncoderRatio(), wrist_pot_translate, false,
                   values.wrist.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> wrist_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &wrist);

      // Stilts
      frc971::PotAndAbsolutePositionT stilts;
      CopyPosition(stilts_encoder_, &stilts,
                   Values::kStiltsEncoderCountsPerRevolution(),
                   Values::kStiltsEncoderRatio(), stilts_pot_translate, false,
                   values.stilts.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> stilts_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &stilts);

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();

      position_builder.add_elevator(elevator_offset);
      position_builder.add_intake_joint(intake_joint_offset);
      position_builder.add_wrist(wrist_offset);
      position_builder.add_stilts(stilts_offset);

      // Suction
      constexpr float kMinVoltage = 0.5;
      constexpr float kMaxVoltage = 2.1;
      position_builder.add_suction_pressure(
          (vacuum_sensor_->GetVoltage() - kMinVoltage) /
          (kMaxVoltage - kMinVoltage));

      position_builder.add_platform_left_detect(!platform_left_detect_->Get());
      position_builder.add_platform_right_detect(
          !platform_right_detect_->Get());

      builder.Send(position_builder.Finish());
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

      builder.Send(auto_mode_builder.Finish());
    }
  }

 private:
  ::aos::Sender<::frc971::autonomous::AutonomousMode> auto_mode_sender_;
  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer elevator_encoder_,
      wrist_encoder_, stilts_encoder_;

  ::std::unique_ptr<frc::DigitalInput> platform_left_detect_;
  ::std::unique_ptr<frc::DigitalInput> platform_right_detect_;

  ::std::unique_ptr<frc::AnalogInput> vacuum_sensor_;

  ::std::array<::std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  ::frc971::wpilib::AbsoluteEncoder intake_encoder_;
};

class CameraReader {
 public:
  CameraReader(::aos::EventLoop *event_loop)
      : camera_frame_sender_(
            event_loop
                ->MakeSender<::y2019::control_loops::drivetrain::CameraFrame>(
                    "/camera")),
        camera_log_fetcher_(
            event_loop->MakeFetcher<::y2019::CameraLog>("/camera")) {}

  CameraReader(const CameraReader &) = delete;
  CameraReader &operator=(const CameraReader &) = delete;

  void set_spi(frc::SPI *spi) {
    spi_ = spi;
    spi_->SetClockRate(1e6);
    spi_->SetChipSelectActiveHigh();
    spi_->SetClockActiveLow();
    spi_->SetSampleDataOnFalling();
    // It ignores you if you try changing this...
    spi_->SetMSBFirst();
  }

  void set_activate_usb(std::unique_ptr<frc::DigitalInput> activate_usb) {
    activate_usb_ = std::move(activate_usb);
  }

  void set_activate_passthrough(
      std::unique_ptr<frc::DigitalInput> activate_passthrough) {
    activate_passthrough_ = std::move(activate_passthrough);
  }

  void DoSpiTransaction() {
    using namespace frc971::jevois;
    RoborioToTeensy to_teensy{};
    to_teensy.realtime_now = aos::realtime_clock::now();
    camera_log_fetcher_.Fetch();
    if (activate_usb_ && !activate_usb_->Get()) {
      to_teensy.camera_command = CameraCommand::kUsb;
    } else if (activate_passthrough_ && !activate_passthrough_->Get()) {
      to_teensy.camera_command = CameraCommand::kCameraPassthrough;
    } else if (camera_log_fetcher_.get() && camera_log_fetcher_->log()) {
      to_teensy.camera_command = CameraCommand::kLog;
    } else {
      to_teensy.camera_command = CameraCommand::kNormal;
    }

    std::array<char, spi_transfer_size() + 1> to_send{};
    {
      const auto to_send_data =
          gsl::make_span(to_send).last<spi_transfer_size()>();
      const auto encoded = SpiPackToTeensy(to_teensy);
      std::copy(encoded.begin(), encoded.end(), to_send_data.begin());
    }
    rx_clearer_.ClearRxFifo();
    // First, send recieve a dummy byte because the Teensy can't control what it
    // sends for the first byte.
    std::array<char, spi_transfer_size() + 1> to_receive;
    DoTransaction(to_send, to_receive);
    const auto unpacked = SpiUnpackToRoborio(
        gsl::make_span(to_receive).last(spi_transfer_size()));
    if (!unpacked) {
      AOS_LOG(INFO, "Decoding SPI data failed\n");
      return;
    }

    const aos::monotonic_clock::time_point now = aos::monotonic_clock::now();
    for (const auto &received : unpacked->frames) {
      auto builder = camera_frame_sender_.MakeBuilder();

      std::array<
          flatbuffers::Offset<y2019::control_loops::drivetrain::CameraTarget>,
          3>
          targets;

      for (size_t i = 0; i < received.targets.size(); ++i) {
        y2019::control_loops::drivetrain::CameraTarget::Builder
            camera_target_builder = builder.MakeBuilder<
                y2019::control_loops::drivetrain::CameraTarget>();

        camera_target_builder.add_distance(received.targets[i].distance);
        camera_target_builder.add_height(received.targets[i].height);
        camera_target_builder.add_heading(received.targets[i].heading);
        camera_target_builder.add_skew(received.targets[i].skew);

        targets[i] = camera_target_builder.Finish();
      }

      flatbuffers::Offset<flatbuffers::Vector<
          flatbuffers::Offset<y2019::control_loops::drivetrain::CameraTarget>>>
          targets_offset = builder.fbb()->CreateVector(targets.begin(),
                                                       received.targets.size());

      y2019::control_loops::drivetrain::CameraFrame::Builder
          camera_frame_builder =
              builder
                  .MakeBuilder<y2019::control_loops::drivetrain::CameraFrame>();

      camera_frame_builder.add_targets(targets_offset);

      // Add an extra 10ms delay to account for unmodeled delays that Austin
      // thinks exists.
      camera_frame_builder.add_timestamp(
          std::chrono::nanoseconds(
              (now - received.age - ::std::chrono::milliseconds(10))
                  .time_since_epoch())
              .count());
      camera_frame_builder.add_camera(received.camera_index);
      builder.Send(camera_frame_builder.Finish());
    }

    if (dummy_spi_) {
      uint8_t dummy_send, dummy_receive;
      dummy_spi_->Transaction(&dummy_send, &dummy_receive, 1);
    }
  }

  void DoTransaction(gsl::span<char> to_send, gsl::span<char> to_receive) {
    AOS_CHECK_EQ(to_send.size(), to_receive.size());
    const auto result = spi_->Transaction(
        reinterpret_cast<uint8_t *>(to_send.data()),
        reinterpret_cast<uint8_t *>(to_receive.data()), to_send.size());
    if (result == to_send.size()) {
      return;
    }
    if (result == -1) {
      AOS_LOG(INFO, "SPI::Transaction of %zd bytes failed\n", to_send.size());
      return;
    }
    AOS_LOG(FATAL, "SPI::Transaction returned something weird\n");
  }

  void SetDummySPI(frc::SPI::Port port) {
    dummy_spi_.reset(new frc::SPI(port));
    // Pick the same settings here in case the roboRIO decides to try something
    // stupid when switching.
    if (dummy_spi_) {
      dummy_spi_->SetClockRate(1e5);
      dummy_spi_->SetChipSelectActiveLow();
      dummy_spi_->SetClockActiveLow();
      dummy_spi_->SetSampleDataOnFalling();
      dummy_spi_->SetMSBFirst();
    }
  }

 private:
  ::aos::Sender<::y2019::control_loops::drivetrain::CameraFrame>
      camera_frame_sender_;
  ::aos::Fetcher<::y2019::CameraLog> camera_log_fetcher_;

  frc::SPI *spi_ = nullptr;
  ::std::unique_ptr<frc::SPI> dummy_spi_;

  std::unique_ptr<frc::DigitalInput> activate_usb_;
  std::unique_ptr<frc::DigitalInput> activate_passthrough_;

  frc971::wpilib::SpiRxClearer rx_clearer_;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure"),
        robot_state_fetcher_(
            event_loop->MakeFetcher<::aos::RobotState>("/aos")) {}

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
  void Write(const superstructure::Output &output) override {
    elevator_victor_->SetSpeed(::aos::Clip(output.elevator_voltage(),
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                               12.0);

    intake_victor_->SetSpeed(::aos::Clip(output.intake_joint_voltage(),
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    wrist_victor_->SetSpeed(::aos::Clip(-output.wrist_voltage(),
                                        -kMaxBringupPower, kMaxBringupPower) /
                            12.0);

    stilts_victor_->SetSpeed(::aos::Clip(output.stilts_voltage(),
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    robot_state_fetcher_.Fetch();
    const double battery_voltage = robot_state_fetcher_.get()
                                       ? robot_state_fetcher_->voltage_battery()
                                       : 12.0;

    // Throw a fast low pass filter on the battery voltage so we don't respond
    // too fast to noise.
    filtered_battery_voltage_ =
        0.5 * filtered_battery_voltage_ + 0.5 * battery_voltage;

    suction_victor_->SetSpeed(::aos::Clip(
        output.pump_voltage() / filtered_battery_voltage_, -1.0, 1.0));
  }

  void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");

    elevator_victor_->SetDisabled();
    intake_victor_->SetDisabled();
    wrist_victor_->SetDisabled();
    stilts_victor_->SetDisabled();
    suction_victor_->SetDisabled();
  }

  ::aos::Fetcher<::aos::RobotState> robot_state_fetcher_;

  ::std::unique_ptr<::frc::VictorSP> elevator_victor_, intake_victor_,
      wrist_victor_, stilts_victor_, suction_victor_;

  double filtered_battery_voltage_ = 12.0;
};

class SolenoidWriter {
 public:
  SolenoidWriter(::aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        superstructure_fetcher_(
            event_loop->MakeFetcher<superstructure::Output>("/superstructure")),
        status_light_fetcher_(
            event_loop->MakeFetcher<::y2019::StatusLight>("/superstructure")),
        pneumatics_to_log_sender_(
            event_loop->MakeSender<::frc971::wpilib::PneumaticsToLog>("/aos")) {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                               ::std::chrono::milliseconds(20),
                               ::std::chrono::milliseconds(1));
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
    intake_rollers_talon_->ConfigContinuousCurrentLimit(10.0, 0);
    intake_rollers_talon_->EnableCurrentLimit(true);
  }

  void Loop(const int iterations) {
    if (iterations != 1) {
      AOS_LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    {
      superstructure_fetcher_.Fetch();
      if (superstructure_fetcher_.get()) {
        big_suction_cup0_->Set(
            !superstructure_fetcher_->intake_suction_bottom());
        big_suction_cup1_->Set(
            !superstructure_fetcher_->intake_suction_bottom());
        small_suction_cup0_->Set(superstructure_fetcher_->intake_suction_top());
        small_suction_cup1_->Set(superstructure_fetcher_->intake_suction_top());

        intake_rollers_talon_->Set(
            ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
            ::aos::Clip(superstructure_fetcher_->intake_roller_voltage(),
                        -kMaxBringupPower, kMaxBringupPower) /
                12.0);
      }
    }

    {
      auto builder = pneumatics_to_log_sender_.MakeBuilder();

      ::frc971::wpilib::PneumaticsToLog::Builder to_log_builder =
          builder.MakeBuilder<frc971::wpilib::PneumaticsToLog>();

      pcm_.Flush();
      to_log_builder.add_read_solenoids(pcm_.GetAll());
      builder.Send(to_log_builder.Finish());
    }

    status_light_fetcher_.Fetch();
    // If we don't have a light request (or it's an old one), we are borked.
    // Flash the red light slowly.
    StatusLightT color;
    if (!status_light_fetcher_.get() ||
        status_light_fetcher_.context().monotonic_event_time +
                chrono::milliseconds(100) <
            event_loop_->monotonic_now()) {
      color.red = 0.0;
      color.green = 0.0;
      color.blue = 0.0;

      ++light_flash_;
      if (light_flash_ > 10) {
        color.red = 0.5;
      }

      if (light_flash_ > 20) {
        light_flash_ = 0;
      }

    } else {
      status_light_fetcher_->UnPackTo(&color);
    }
    SetColor(color);
  }

  void SetColor(const StatusLightT status_light) {
    // Save CAN bandwidth and CPU at the cost of RT.  Only change the light when
    // it actually changes.  This is pretty low priority anyways.
    static int time_since_last_send = 0;
    ++time_since_last_send;
    if (time_since_last_send > 10) {
      time_since_last_send = 0;
    }
    if (status_light.green != last_green_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(status_light.green,
                             ::ctre::phoenix::CANifier::LEDChannelA);
      last_green_ = status_light.green;
    }

    if (status_light.blue != last_blue_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(status_light.blue,
                             ::ctre::phoenix::CANifier::LEDChannelC);
      last_blue_ = status_light.blue;
    }

    if (status_light.red != last_red_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(status_light.red,
                             ::ctre::phoenix::CANifier::LEDChannelB);
      last_red_ = status_light.red;
    }
  }

 private:
  ::aos::EventLoop *event_loop_;

  ::frc971::wpilib::BufferedPcm pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> big_suction_cup0_,
      big_suction_cup1_, small_suction_cup0_, small_suction_cup1_;

  ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonSRX>
      intake_rollers_talon_;

  ::aos::Fetcher<::y2019::control_loops::superstructure::Output>
      superstructure_fetcher_;
  ::aos::Fetcher<::y2019::StatusLight> status_light_fetcher_;

  aos::Sender<::frc971::wpilib::PneumaticsToLog> pneumatics_to_log_sender_;

  ::ctre::phoenix::CANifier canifier_{0};

  double last_red_ = -1.0;
  double last_green_ = -1.0;
  double last_blue_ = -1.0;

  int light_flash_ = 0;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run() override {
    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("config.json");

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

    sensor_reader.set_elevator_encoder(make_encoder(4));
    sensor_reader.set_elevator_absolute_pwm(make_unique<frc::DigitalInput>(4));
    sensor_reader.set_elevator_potentiometer(make_unique<frc::AnalogInput>(4));

    sensor_reader.set_wrist_encoder(make_encoder(5));
    sensor_reader.set_wrist_absolute_pwm(make_unique<frc::DigitalInput>(5));
    sensor_reader.set_wrist_potentiometer(make_unique<frc::AnalogInput>(5));

    sensor_reader.set_intake_encoder(make_encoder(2));
    sensor_reader.set_intake_absolute_pwm(make_unique<frc::DigitalInput>(2));

    sensor_reader.set_stilts_encoder(make_encoder(3));
    sensor_reader.set_stilts_absolute_pwm(make_unique<frc::DigitalInput>(3));
    sensor_reader.set_stilts_potentiometer(make_unique<frc::AnalogInput>(3));

    sensor_reader.set_pwm_trigger(true);
    sensor_reader.set_vacuum_sensor(7);

    sensor_reader.set_platform_right_detect(make_unique<frc::DigitalInput>(6));
    sensor_reader.set_platform_left_detect(make_unique<frc::DigitalInput>(7));

    sensor_reader.set_autonomous_mode(0, make_unique<frc::DigitalInput>(22));
    sensor_reader.set_autonomous_mode(0, make_unique<frc::DigitalInput>(23));
    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop imu_event_loop(&config.message());
    CameraReader camera_reader(&imu_event_loop);
    frc::SPI camera_spi(frc::SPI::Port::kOnboardCS3);
    camera_reader.set_spi(&camera_spi);
    camera_reader.SetDummySPI(frc::SPI::Port::kOnboardCS2);
    // Austin says 8, 9, 24, and 25 are good options to choose from for these.
    camera_reader.set_activate_usb(make_unique<frc::DigitalInput>(24));
    camera_reader.set_activate_passthrough(make_unique<frc::DigitalInput>(25));

    auto imu_trigger = make_unique<frc::DigitalInput>(0);
    ::frc971::wpilib::ADIS16448 imu(&imu_event_loop, frc::SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.set_spi_idle_callback(
        [&camera_reader]() { camera_reader.DoSpiTransaction(); });
    auto imu_reset = make_unique<frc::DigitalOutput>(1);
    imu.set_reset(imu_reset.get());
    AddLoop(&imu_event_loop);

    // While as of 2/9/18 the drivetrain Victors are SPX, it appears as though
    // they are identical, as far as DrivetrainWriter is concerned, to the SP
    // variety so all the Victors are written as SPs.

    // Thread 5.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), false);

    SuperstructureWriter superstructure_writer(&output_event_loop);
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
    AddLoop(&output_event_loop);

    // Thread 6.
    ::aos::ShmEventLoop solenoid_writer_event_loop(&config.message());
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop);
    solenoid_writer.set_intake_roller_talon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonSRX>(10));
    solenoid_writer.set_big_suction_cup(0, 1);
    solenoid_writer.set_small_suction_cup(2, 3);
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2019

AOS_ROBOT_CLASS(::y2019::wpilib::WPILibRobot);
