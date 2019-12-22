#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Talon.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/events/shm_event_loop.h"
#include "aos/logging/logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/autonomous/auto_generated.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/shooter/shooter_output_generated.h"
#include "y2016/control_loops/shooter/shooter_position_generated.h"
#include "y2016/control_loops/superstructure/superstructure_output_generated.h"
#include "y2016/control_loops/superstructure/superstructure_position_generated.h"
#include "y2016/queues/ball_detector_generated.h"

using aos::make_unique;
using ::frc971::wpilib::LoopOutputHandler;
namespace shooter = ::y2016::control_loops::shooter;
namespace superstructure = ::y2016::control_loops::superstructure;

namespace y2016 {
namespace wpilib {
namespace {
constexpr double kMaxBringupPower = 12.0;
}  // namespace

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

// Translates for the sensor values to convert raw index pulses into something
// with proper units.

// TODO(comran): Template these methods since there is a lot of repetition here.
double hall_translate(double in) {
  // Turn voltage from our 3-state halls into a ratio that the loop can use.
  return in / 5.0;
}

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         constants::Values::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double shooter_translate(int32_t in) {
  return -static_cast<double>(in) / (128.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kShooterEncoderRatio * (2 * M_PI /*radians*/);
}

double intake_translate(int32_t in) {
  return static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kIntakeEncoderRatio * (2 * M_PI /*radians*/);
}

double shoulder_translate(int32_t in) {
  return -static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kShoulderEncoderRatio * (2 * M_PI /*radians*/);
}

double wrist_translate(int32_t in) {
  return -static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kWristEncoderRatio * (2 * M_PI /*radians*/);
}

double intake_pot_translate(double voltage) {
  return voltage * constants::Values::kIntakePotRatio *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double shoulder_pot_translate(double voltage) {
  return voltage * constants::Values::kShoulderPotRatio *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double wrist_pot_translate(double voltage) {
  return voltage * constants::Values::kWristPotRatio *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxDrivetrainEncoderPulsesPerSecond =
    5600.0 /* CIM free speed RPM */ * 14.0 / 48.0 /* 1st reduction */ * 28.0 /
    50.0 /* 2nd reduction (high gear) */ * 30.0 / 44.0 /* encoder gears */ /
    60.0 /* seconds per minute */ * 256.0 /* CPR */ * 4 /* edges per cycle */;

constexpr double kMaxShooterEncoderPulsesPerSecond =
    18700.0 /* 775pro free speed RPM */ * 12.0 /
    18.0 /* motor to encoder reduction */ / 60.0 /* seconds per minute */ *
    128.0 /* CPR */ * 4 /* edges per cycle */;

double kMaxDrivetrainShooterEncoderPulsesPerSecond = ::std::max(
    kMaxDrivetrainEncoderPulsesPerSecond, kMaxShooterEncoderPulsesPerSecond);

constexpr double kMaxSuperstructureEncoderPulsesPerSecond =
    18700.0 /* 775pro free speed RPM */ * 12.0 /
    56.0 /* motor to encoder reduction */ / 60.0 /* seconds per minute */ *
    512.0 /* CPR */ * 4 /* index pulse every quarter cycle */;

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        ball_detector_sender_(
            event_loop->MakeSender<::y2016::sensors::BallDetector>(
                "/superstructure")),
        auto_mode_sender_(
            event_loop->MakeSender<::frc971::autonomous::AutonomousMode>(
                "/aos")),
        shooter_position_sender_(
            event_loop->MakeSender<shooter::Position>("/shooter")),
        superstructure_position_sender_(
            event_loop->MakeSender<superstructure::Position>(
                "/superstructure")),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                    "/drivetrain")) {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxDrivetrainShooterEncoderPulsesPerSecond);
    UpdateMediumEncoderFilterHz(kMaxSuperstructureEncoderPulsesPerSecond);
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  void set_drivetrain_left_hall(::std::unique_ptr<::frc::AnalogInput> analog) {
    drivetrain_left_hall_ = ::std::move(analog);
  }

  void set_drivetrain_right_hall(::std::unique_ptr<::frc::AnalogInput> analog) {
    drivetrain_right_hall_ = ::std::move(analog);
  }

  // Shooter setters.
  void set_shooter_left_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    shooter_left_encoder_ = ::std::move(encoder);
  }

  void set_shooter_right_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    shooter_right_encoder_ = ::std::move(encoder);
  }

  // Intake setters.
  void set_intake_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_potentiometer(
      ::std::unique_ptr<::frc::AnalogInput> potentiometer) {
    intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_index(::std::unique_ptr<::frc::DigitalInput> index) {
    medium_encoder_filter_.Add(index.get());
    intake_encoder_.set_index(::std::move(index));
  }

  // Shoulder setters.
  void set_shoulder_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    shoulder_encoder_.set_encoder(::std::move(encoder));
  }

  void set_shoulder_potentiometer(
      ::std::unique_ptr<::frc::AnalogInput> potentiometer) {
    shoulder_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_shoulder_index(::std::unique_ptr<::frc::DigitalInput> index) {
    medium_encoder_filter_.Add(index.get());
    shoulder_encoder_.set_index(::std::move(index));
  }

  // Wrist setters.
  void set_wrist_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
  }

  void set_wrist_potentiometer(
      ::std::unique_ptr<::frc::AnalogInput> potentiometer) {
    wrist_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_wrist_index(::std::unique_ptr<::frc::DigitalInput> index) {
    medium_encoder_filter_.Add(index.get());
    wrist_encoder_.set_index(::std::move(index));
  }

  // Ball detector setter.
  void set_ball_detector(::std::unique_ptr<::frc::AnalogInput> analog) {
    ball_detector_ = ::std::move(analog);
  }

  // Autonomous mode switch setter.
  void set_autonomous_mode(int i,
                           ::std::unique_ptr<::frc::DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.

  void Start() {
    AddToDMA(&intake_encoder_);
    AddToDMA(&shoulder_encoder_);
    AddToDMA(&wrist_encoder_);
  }

  void RunIteration() {
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();
      frc971::control_loops::drivetrain::Position::Builder position_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();

      position_builder.add_right_encoder(
          drivetrain_translate(-drivetrain_right_encoder_->GetRaw()));
      position_builder.add_left_encoder(
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw()));
      position_builder.add_left_speed(
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));
      position_builder.add_right_speed(drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod()));

      position_builder.add_left_shifter_position(
          hall_translate(drivetrain_left_hall_->GetVoltage()));
      position_builder.add_right_shifter_position(
          hall_translate(drivetrain_right_hall_->GetVoltage()));

      builder.Send(position_builder.Finish());
    }
  }

  void RunDmaIteration() {
    const auto &values = constants::GetValues();
    {
      auto builder = shooter_position_sender_.MakeBuilder();
      shooter::Position::Builder shooter_builder =
          builder.MakeBuilder<shooter::Position>();
      shooter_builder.add_theta_left(
          shooter_translate(-shooter_left_encoder_->GetRaw()));
      shooter_builder.add_theta_right(
          shooter_translate(shooter_right_encoder_->GetRaw()));
      builder.Send(shooter_builder.Finish());
    }

    {
      auto builder = superstructure_position_sender_.MakeBuilder();

      frc971::PotAndIndexPositionT intake;
      CopyPosition(intake_encoder_, &intake, intake_translate,
                   intake_pot_translate, false, values.intake.pot_offset);
      flatbuffers::Offset<frc971::PotAndIndexPosition> intake_offset =
          frc971::PotAndIndexPosition::Pack(*builder.fbb(), &intake);

      frc971::PotAndIndexPositionT shoulder;
      CopyPosition(shoulder_encoder_, &shoulder, shoulder_translate,
                   shoulder_pot_translate, false, values.shoulder.pot_offset);
      flatbuffers::Offset<frc971::PotAndIndexPosition> shoulder_offset =
          frc971::PotAndIndexPosition::Pack(*builder.fbb(), &shoulder);

      frc971::PotAndIndexPositionT wrist;
      CopyPosition(wrist_encoder_, &wrist, wrist_translate, wrist_pot_translate,
                   true, values.wrist.pot_offset);
      flatbuffers::Offset<frc971::PotAndIndexPosition> wrist_offset =
          frc971::PotAndIndexPosition::Pack(*builder.fbb(), &wrist);

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();

      position_builder.add_intake(intake_offset);
      position_builder.add_shoulder(shoulder_offset);
      position_builder.add_wrist(wrist_offset);

      builder.Send(position_builder.Finish());
    }

    {
      auto builder = ball_detector_sender_.MakeBuilder();
      ::y2016::sensors::BallDetector::Builder ball_detector_builder =
          builder.MakeBuilder<y2016::sensors::BallDetector>();
      ball_detector_builder.add_voltage(ball_detector_->GetVoltage());
      builder.Send(ball_detector_builder.Finish());
    }

    {
      auto builder = auto_mode_sender_.MakeBuilder();
      ::frc971::autonomous::AutonomousMode::Builder auto_builder =
          builder.MakeBuilder<frc971::autonomous::AutonomousMode>();
      int mode = 0;
      for (size_t i = 0; i < autonomous_modes_.size(); ++i) {
        if (autonomous_modes_[i]->Get()) {
          mode |= 1 << i;
        }
      }
      auto_builder.add_mode(mode);
      builder.Send(auto_builder.Finish());
    }
  }

 private:
  ::aos::Sender<::y2016::sensors::BallDetector> ball_detector_sender_;
  ::aos::Sender<::frc971::autonomous::AutonomousMode> auto_mode_sender_;
  ::aos::Sender<shooter::Position> shooter_position_sender_;
  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;

  ::std::unique_ptr<::frc::AnalogInput> drivetrain_left_hall_,
      drivetrain_right_hall_;

  ::std::unique_ptr<::frc::Encoder> shooter_left_encoder_,
      shooter_right_encoder_;
  ::frc971::wpilib::DMAEncoderAndPotentiometer intake_encoder_,
      shoulder_encoder_, wrist_encoder_;
  ::std::unique_ptr<::frc::AnalogInput> ball_detector_;

  ::std::array<::std::unique_ptr<::frc::DigitalInput>, 4> autonomous_modes_;

  ::frc::DigitalGlitchFilter hall_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(::aos::ShmEventLoop *event_loop,
                 ::frc971::wpilib::BufferedPcm *pcm)
      : pcm_(pcm),
        pneumatics_to_log_sender_(
            event_loop->MakeSender<::frc971::wpilib::PneumaticsToLog>("/aos")),
        drivetrain_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::drivetrain::Output>(
                    "/drivetrain")),
        shooter_(event_loop->MakeFetcher<shooter::Output>("/shooter")),
        superstructure_(
            event_loop
                ->MakeFetcher<::y2016::control_loops::superstructure::Output>(
                    "/superstructure")) {
    event_loop->set_name("Solenoids");
    event_loop->SetRuntimeRealtimePriority(27);

    event_loop->OnRun([this]() { compressor_->Start(); });

    event_loop->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                              ::std::chrono::milliseconds(20),
                              ::std::chrono::milliseconds(1));
  }

  void set_compressor(::std::unique_ptr<::frc::Compressor> compressor) {
    compressor_ = ::std::move(compressor);
  }

  void set_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_shifter_ = ::std::move(s);
  }

  void set_climber_trigger(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    climber_trigger_ = ::std::move(s);
  }

  void set_traverse(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    traverse_ = ::std::move(s);
  }

  void set_traverse_latch(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    traverse_latch_ = ::std::move(s);
  }

  void set_shooter_clamp(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    shooter_clamp_ = ::std::move(s);
  }

  void set_shooter_pusher(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    shooter_pusher_ = ::std::move(s);
  }

  void set_lights(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    lights_ = ::std::move(s);
  }

  void set_flashlight(::std::unique_ptr<::frc::Relay> relay) {
    flashlight_ = ::std::move(relay);
  }

 private:
  void Loop(const int iterations) {
    if (iterations != 1) {
      AOS_LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    {
      drivetrain_.Fetch();
      if (drivetrain_.get()) {
        drivetrain_shifter_->Set(
            !(drivetrain_->left_high() || drivetrain_->right_high()));
      }
    }

    {
      shooter_.Fetch();
      if (shooter_.get()) {
        shooter_clamp_->Set(shooter_->clamp_open());
        shooter_pusher_->Set(shooter_->push_to_shooter());
        lights_->Set(shooter_->lights_on());
        if (shooter_->forwards_flashlight()) {
          if (shooter_->backwards_flashlight()) {
            flashlight_->Set(::frc::Relay::kOn);
          } else {
            flashlight_->Set(::frc::Relay::kReverse);
          }
        } else {
          if (shooter_->backwards_flashlight()) {
            flashlight_->Set(::frc::Relay::kForward);
          } else {
            flashlight_->Set(::frc::Relay::kOff);
          }
        }
      }
    }

    {
      superstructure_.Fetch();
      if (superstructure_.get()) {
        climber_trigger_->Set(superstructure_->unfold_climber());

        traverse_->Set(superstructure_->traverse_down());
        traverse_latch_->Set(superstructure_->traverse_unlatched());
      }
    }

    {
      auto builder = pneumatics_to_log_sender_.MakeBuilder();

      ::frc971::wpilib::PneumaticsToLog::Builder to_log_builder =
          builder.MakeBuilder<frc971::wpilib::PneumaticsToLog>();

      to_log_builder.add_compressor_on(compressor_->Enabled());

      pcm_->Flush();
      to_log_builder.add_read_solenoids(pcm_->GetAll());
      builder.Send(to_log_builder.Finish());
    }
  }

  ::frc971::wpilib::BufferedPcm *pcm_;
  aos::Sender<::frc971::wpilib::PneumaticsToLog> pneumatics_to_log_sender_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_shifter_,
      shooter_clamp_, shooter_pusher_, lights_, traverse_, traverse_latch_,
      climber_trigger_;
  ::std::unique_ptr<::frc::Relay> flashlight_;
  ::std::unique_ptr<::frc::Compressor> compressor_;

  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output> drivetrain_;
  ::aos::Fetcher<shooter::Output> shooter_;
  ::aos::Fetcher<::y2016::control_loops::superstructure::Output>
      superstructure_;
};

class ShooterWriter : public LoopOutputHandler<shooter::Output> {
 public:
  ShooterWriter(::aos::EventLoop *event_loop)
      : LoopOutputHandler<shooter::Output>(event_loop, "/shooter") {}

  void set_shooter_left_talon(::std::unique_ptr<::frc::Talon> t) {
    shooter_left_talon_ = ::std::move(t);
  }

  void set_shooter_right_talon(::std::unique_ptr<::frc::Talon> t) {
    shooter_right_talon_ = ::std::move(t);
  }

 private:
  void Write(const shooter::Output &output) override {
    shooter_left_talon_->SetSpeed(output.voltage_left() / 12.0);
    shooter_right_talon_->SetSpeed(-output.voltage_right() / 12.0);
  }

  void Stop() override {
    AOS_LOG(WARNING, "Shooter output too old.\n");
    shooter_left_talon_->SetDisabled();
    shooter_right_talon_->SetDisabled();
  }

  ::std::unique_ptr<::frc::Talon> shooter_left_talon_, shooter_right_talon_;
};

class SuperstructureWriter : public LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : LoopOutputHandler<superstructure::Output>(event_loop,
                                                  "/superstructure") {}

  void set_intake_talon(::std::unique_ptr<::frc::Talon> t) {
    intake_talon_ = ::std::move(t);
  }

  void set_shoulder_talon(::std::unique_ptr<::frc::Talon> t) {
    shoulder_talon_ = ::std::move(t);
  }

  void set_wrist_talon(::std::unique_ptr<::frc::Talon> t) {
    wrist_talon_ = ::std::move(t);
  }

  void set_top_rollers_talon(::std::unique_ptr<::frc::Talon> t) {
    top_rollers_talon_ = ::std::move(t);
  }

  void set_bottom_rollers_talon(::std::unique_ptr<::frc::Talon> t) {
    bottom_rollers_talon_ = ::std::move(t);
  }

  void set_climber_talon(::std::unique_ptr<::frc::Talon> t) {
    climber_talon_ = ::std::move(t);
  }

 private:
  virtual void Write(const superstructure::Output &output) override {
    intake_talon_->SetSpeed(::aos::Clip(output.voltage_intake(),
                                        -kMaxBringupPower, kMaxBringupPower) /
                            12.0);
    shoulder_talon_->SetSpeed(::aos::Clip(-output.voltage_shoulder(),
                                          -kMaxBringupPower, kMaxBringupPower) /
                              12.0);
    wrist_talon_->SetSpeed(::aos::Clip(output.voltage_wrist(),
                                       -kMaxBringupPower, kMaxBringupPower) /
                           12.0);
    top_rollers_talon_->SetSpeed(-output.voltage_top_rollers() / 12.0);
    bottom_rollers_talon_->SetSpeed(-output.voltage_bottom_rollers() / 12.0);
    climber_talon_->SetSpeed(-output.voltage_climber() / 12.0);
  }

  virtual void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    intake_talon_->SetDisabled();
    shoulder_talon_->SetDisabled();
    wrist_talon_->SetDisabled();
  }

  ::std::unique_ptr<::frc::Talon> intake_talon_, shoulder_talon_, wrist_talon_,
      top_rollers_talon_, bottom_rollers_talon_, climber_talon_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<::frc::Encoder> make_encoder(int index) {
    return make_unique<::frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                       ::frc::Encoder::k4X);
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

    sensor_reader.set_drivetrain_left_encoder(make_encoder(5));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(6));
    sensor_reader.set_drivetrain_left_hall(make_unique<::frc::AnalogInput>(5));
    sensor_reader.set_drivetrain_right_hall(make_unique<::frc::AnalogInput>(6));

    sensor_reader.set_shooter_left_encoder(make_encoder(7));
    sensor_reader.set_shooter_right_encoder(make_encoder(-3));

    sensor_reader.set_intake_encoder(make_encoder(0));
    sensor_reader.set_intake_index(make_unique<::frc::DigitalInput>(0));
    sensor_reader.set_intake_potentiometer(make_unique<::frc::AnalogInput>(0));

    sensor_reader.set_shoulder_encoder(make_encoder(4));
    sensor_reader.set_shoulder_index(make_unique<::frc::DigitalInput>(2));
    sensor_reader.set_shoulder_potentiometer(
        make_unique<::frc::AnalogInput>(2));

    sensor_reader.set_wrist_encoder(make_encoder(1));
    sensor_reader.set_wrist_index(make_unique<::frc::DigitalInput>(1));
    sensor_reader.set_wrist_potentiometer(make_unique<::frc::AnalogInput>(1));

    sensor_reader.set_ball_detector(make_unique<::frc::AnalogInput>(7));

    sensor_reader.set_autonomous_mode(0, make_unique<::frc::DigitalInput>(9));
    sensor_reader.set_autonomous_mode(1, make_unique<::frc::DigitalInput>(8));
    sensor_reader.set_autonomous_mode(2, make_unique<::frc::DigitalInput>(7));
    sensor_reader.set_autonomous_mode(3, make_unique<::frc::DigitalInput>(6));
    AddLoop(&sensor_reader_event_loop);

    // TODO(Brian): This interacts poorly with the SpiRxClearer in ADIS16448.
    // Thread 4.
    ::aos::ShmEventLoop gyro_event_loop(&config.message());
    ::frc971::wpilib::GyroSender gyro_sender(&gyro_event_loop);
    AddLoop(&gyro_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop imu_event_loop(&config.message());
    auto imu_trigger = make_unique<::frc::DigitalInput>(3);
    ::frc971::wpilib::ADIS16448 imu(&imu_event_loop, ::frc::SPI::Port::kMXP,
                                    imu_trigger.get());
    AddLoop(&imu_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(5)), false);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(4)), true);

    ShooterWriter shooter_writer(&output_event_loop);
    shooter_writer.set_shooter_left_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(9)));
    shooter_writer.set_shooter_right_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(8)));

    SuperstructureWriter superstructure_writer(&output_event_loop);
    superstructure_writer.set_intake_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(3)));
    superstructure_writer.set_shoulder_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(6)));
    superstructure_writer.set_wrist_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(2)));
    superstructure_writer.set_top_rollers_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(1)));
    superstructure_writer.set_bottom_rollers_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(0)));
    superstructure_writer.set_climber_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(7)));

    AddLoop(&output_event_loop);

    // Thread 6.
    ::aos::ShmEventLoop solenoid_writer_event_loop(&config.message());
    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop, pcm.get());
    solenoid_writer.set_drivetrain_shifter(pcm->MakeSolenoid(0));
    solenoid_writer.set_traverse_latch(pcm->MakeSolenoid(2));
    solenoid_writer.set_traverse(pcm->MakeSolenoid(3));
    solenoid_writer.set_shooter_clamp(pcm->MakeSolenoid(4));
    solenoid_writer.set_shooter_pusher(pcm->MakeSolenoid(5));
    solenoid_writer.set_lights(pcm->MakeSolenoid(6));
    solenoid_writer.set_climber_trigger(pcm->MakeSolenoid(1));
    solenoid_writer.set_flashlight(make_unique<::frc::Relay>(0));

    solenoid_writer.set_compressor(make_unique<::frc::Compressor>());
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2016

AOS_ROBOT_CLASS(::y2016::wpilib::WPILibRobot);
