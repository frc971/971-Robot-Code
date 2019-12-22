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
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Servo.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/autonomous/auto_generated.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/superstructure_output_generated.h"
#include "y2017/control_loops/superstructure/superstructure_position_generated.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace superstructure = ::y2017::control_loops::superstructure;
using ::y2017::constants::Values;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
using namespace frc;
using aos::make_unique;

namespace y2017 {
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
  return static_cast<double>(in) /
         Values::kDrivetrainEncoderCountsPerRevolution *
         Values::kDrivetrainEncoderRatio * 2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / Values::kDrivetrainCyclesPerRevolution *
         Values::kDrivetrainEncoderRatio * 2.0 * M_PI;
}

// TODO(Travis): Make sure the number of turns is right.
double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakePotRatio * (3.0 /*turns*/ / 5.0 /*volts*/) *
         (2 * M_PI /*radians*/);
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(Values::kMaxDrivetrainEncoderPulsesPerSecond,
        Values::kMaxShooterEncoderPulsesPerSecond);
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(Values::kMaxIntakeEncoderPulsesPerSecond,
        Values::kMaxTurretEncoderPulsesPerSecond,
        Values::kMaxIndexerEncoderPulsesPerSecond);
static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");
constexpr double kMaxSlowEncoderPulsesPerSecond =
    Values::kMaxHoodEncoderPulsesPerSecond;
static_assert(kMaxSlowEncoderPulsesPerSecond <= 100000,
              "slow encoders are too fast");
static_assert(kMaxSlowEncoderPulsesPerSecond < kMaxMediumEncoderPulsesPerSecond,
              "slow encoders are faster than medium?");

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        auto_mode_sender_(
            event_loop->MakeSender<::frc971::autonomous::AutonomousMode>(
                "/aos")),
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
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  void set_shooter_encoder(::std::unique_ptr<Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    shooter_encoder_ = ::std::move(encoder);
  }

  void set_intake_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_absolute(::std::unique_ptr<DigitalInput> input) {
    intake_encoder_.set_absolute_pwm(::std::move(input));
  }

  void set_indexer_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    indexer_counter_.set_encoder(encoder.get());
    indexer_encoder_ = ::std::move(encoder);
  }

  void set_indexer_hall(::std::unique_ptr<DigitalInput> input) {
    hall_filter_.Add(input.get());
    indexer_counter_.set_input(input.get());
    indexer_hall_ = ::std::move(input);
  }

  void set_turret_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    turret_counter_.set_encoder(encoder.get());
    turret_encoder_ = ::std::move(encoder);
  }

  void set_turret_hall(::std::unique_ptr<DigitalInput> input) {
    hall_filter_.Add(input.get());
    turret_counter_.set_input(input.get());
    turret_hall_ = ::std::move(input);
  }

  void set_hood_encoder(::std::unique_ptr<Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    hood_encoder_.set_encoder(::std::move(encoder));
  }

  void set_hood_index(::std::unique_ptr<DigitalInput> index) {
    medium_encoder_filter_.Add(index.get());
    hood_encoder_.set_index(::std::move(index));
  }

  void set_autonomous_mode(int i, ::std::unique_ptr<DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void Start() {
    AddToDMA(&indexer_counter_);
    AddToDMA(&hood_encoder_);
    AddToDMA(&turret_counter_);
  }

  void RunIteration() {
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();
      frc971::control_loops::drivetrain::Position::Builder position_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();
      position_builder.add_right_encoder(
          drivetrain_translate(drivetrain_right_encoder_->GetRaw()));
      position_builder.add_right_speed(drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod()));

      position_builder.add_left_encoder(
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw()));
      position_builder.add_left_speed(
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));

      builder.Send(position_builder.Finish());
    }
  }

  void RunDMAIteration() {
    const auto values = constants::GetValues();

    {
      auto builder = superstructure_position_sender_.MakeBuilder();
      frc971::PotAndAbsolutePositionT intake;
      CopyPosition(intake_encoder_, &intake,
                   Values::kIntakeEncoderCountsPerRevolution,
                   Values::kIntakeEncoderRatio, intake_pot_translate, true,
                   values.intake.pot_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &intake);

      frc971::HallEffectAndPositionT indexer;
      CopyPosition(indexer_counter_, &indexer,
                   Values::kIndexerEncoderCountsPerRevolution,
                   Values::kIndexerEncoderRatio, true);
      flatbuffers::Offset<frc971::HallEffectAndPosition> indexer_offset =
          frc971::HallEffectAndPosition::Pack(*builder.fbb(), &indexer);

      frc971::IndexPositionT hood;
      CopyPosition(hood_encoder_, &hood,
                   Values::kHoodEncoderCountsPerRevolution,
                   Values::kHoodEncoderRatio, true);
      flatbuffers::Offset<frc971::IndexPosition> hood_offset =
          frc971::IndexPosition::Pack(*builder.fbb(), &hood);

      frc971::HallEffectAndPositionT turret;
      CopyPosition(turret_counter_, &turret,
                   Values::kTurretEncoderCountsPerRevolution,
                   Values::kTurretEncoderRatio, false);
      flatbuffers::Offset<frc971::HallEffectAndPosition> turret_offset =
          frc971::HallEffectAndPosition::Pack(*builder.fbb(), &turret);

      superstructure::ColumnPosition::Builder column_builder =
          builder.MakeBuilder<superstructure::ColumnPosition>();
      column_builder.add_indexer(indexer_offset);
      column_builder.add_turret(turret_offset);
      flatbuffers::Offset<superstructure::ColumnPosition> column_offset =
          column_builder.Finish();

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();

      position_builder.add_column(column_offset);
      position_builder.add_hood(hood_offset);
      position_builder.add_intake(intake_offset);
      position_builder.add_theta_shooter(
          encoder_translate(shooter_encoder_->GetRaw(),
                            Values::kShooterEncoderCountsPerRevolution,
                            Values::kShooterEncoderRatio));

      builder.Send(position_builder.Finish());
    }

    {
      auto builder = auto_mode_sender_.MakeBuilder();
      ::frc971::autonomous::AutonomousMode::Builder auto_builder =
          builder.MakeBuilder<::frc971::autonomous::AutonomousMode>();

      int mode = 0;
      for (size_t i = 0; i < autonomous_modes_.size(); ++i) {
        if (autonomous_modes_[i] && autonomous_modes_[i]->Get()) {
          mode |= 1 << i;
        }
      }
      auto_builder.add_mode(mode);
      builder.Send(auto_builder.Finish());
    }
  }

 private:
  ::aos::Sender<::frc971::autonomous::AutonomousMode> auto_mode_sender_;
  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;

  DigitalGlitchFilter hall_filter_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer intake_encoder_;

  ::std::unique_ptr<Encoder> indexer_encoder_;
  ::std::unique_ptr<DigitalInput> indexer_hall_;
  ::frc971::wpilib::DMAEdgeCounter indexer_counter_;

  ::std::unique_ptr<Encoder> turret_encoder_;
  ::std::unique_ptr<DigitalInput> turret_hall_;
  ::frc971::wpilib::DMAEdgeCounter turret_counter_;

  ::frc971::wpilib::DMAEncoder hood_encoder_;
  ::std::unique_ptr<Encoder> shooter_encoder_;

  ::std::array<::std::unique_ptr<DigitalInput>, 4> autonomous_modes_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(::aos::ShmEventLoop *event_loop)
      : superstructure_output_fetcher_(
            event_loop->MakeFetcher<superstructure::Output>(
                "/superstructure")) {
    event_loop->set_name("Solenoids");
    event_loop->SetRuntimeRealtimePriority(27);

    event_loop->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                              ::std::chrono::milliseconds(20),
                              ::std::chrono::milliseconds(1));
  }

  ::frc971::wpilib::BufferedPcm *pcm() { return &pcm_; }

  void set_lights(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    lights_ = ::std::move(s);
  }

  void set_rgb_light(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    rgb_lights_ = ::std::move(s);
  }

  void Loop(const int iterations) {
    if (iterations != 1) {
      AOS_LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    {
      superstructure_output_fetcher_.Fetch();
      if (superstructure_output_fetcher_.get()) {
        lights_->Set(superstructure_output_fetcher_->lights_on());
        rgb_lights_->Set(superstructure_output_fetcher_->red_light_on() |
                         superstructure_output_fetcher_->green_light_on() |
                         superstructure_output_fetcher_->blue_light_on());
      }
    }

    pcm_.Flush();
  }

 private:
  ::frc971::wpilib::BufferedPcm pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> lights_, rgb_lights_;

  ::aos::Fetcher<::y2017::control_loops::superstructure::Output>
      superstructure_output_fetcher_;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  void set_intake_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_victor_ = ::std::move(t);
  }
  void set_intake_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_rollers_victor_ = ::std::move(t);
  }

  void set_indexer_victor(::std::unique_ptr<::frc::VictorSP> t) {
    indexer_victor_ = ::std::move(t);
  }
  void set_indexer_roller_victor(::std::unique_ptr<::frc::VictorSP> t) {
    indexer_roller_victor_ = ::std::move(t);
  }

  void set_gear_servo(::std::unique_ptr<::frc::Servo> t) {
    gear_servo_ = ::std::move(t);
  }
  void set_shooter_victor(::std::unique_ptr<::frc::VictorSP> t) {
    shooter_victor_ = ::std::move(t);
  }
  void set_turret_victor(::std::unique_ptr<::frc::VictorSP> t) {
    turret_victor_ = ::std::move(t);
  }
  void set_hood_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hood_victor_ = ::std::move(t);
  }

  void set_red_light(::std::unique_ptr<DigitalOutput> t) {
    red_light_ = ::std::move(t);
  }
  void set_green_light(::std::unique_ptr<DigitalOutput> t) {
    green_light_ = ::std::move(t);
  }
  void set_blue_light(::std::unique_ptr<DigitalOutput> t) {
    blue_light_ = ::std::move(t);
  }

 private:
  virtual void Write(const superstructure::Output &output) override {
    intake_victor_->SetSpeed(::aos::Clip(output.voltage_intake(),
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
    intake_rollers_victor_->SetSpeed(output.voltage_intake_rollers() / 12.0);
    indexer_victor_->SetSpeed(-output.voltage_indexer() / 12.0);
    indexer_roller_victor_->SetSpeed(output.voltage_indexer_rollers() / 12.0);
    turret_victor_->SetSpeed(::aos::Clip(-output.voltage_turret(),
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
    hood_victor_->SetSpeed(::aos::Clip(output.voltage_hood(), -kMaxBringupPower,
                                       kMaxBringupPower) /
                           12.0);
    shooter_victor_->SetSpeed(output.voltage_shooter() / 12.0);

    red_light_->Set(output.red_light_on());
    green_light_->Set(output.green_light_on());
    blue_light_->Set(output.blue_light_on());

    gear_servo_->SetPosition(output.gear_servo());
  }

  virtual void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    intake_victor_->SetDisabled();
    intake_rollers_victor_->SetDisabled();
    indexer_victor_->SetDisabled();
    indexer_roller_victor_->SetDisabled();
    turret_victor_->SetDisabled();
    hood_victor_->SetDisabled();
    shooter_victor_->SetDisabled();

    gear_servo_->SetRaw(0);

    red_light_->Set(true);
    green_light_->Set(true);
    blue_light_->Set(true);
  }

  ::std::unique_ptr<::frc::VictorSP> intake_victor_, intake_rollers_victor_,
      indexer_victor_, indexer_roller_victor_, shooter_victor_, turret_victor_,
      hood_victor_;

  ::std::unique_ptr<::frc::Servo> gear_servo_;

  ::std::unique_ptr<DigitalOutput> red_light_, green_light_, blue_light_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
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

    sensor_reader.set_intake_encoder(make_encoder(3));
    sensor_reader.set_intake_absolute(make_unique<DigitalInput>(0));
    sensor_reader.set_intake_potentiometer(make_unique<AnalogInput>(4));

    sensor_reader.set_indexer_encoder(make_encoder(5));
    sensor_reader.set_indexer_hall(make_unique<DigitalInput>(4));

    sensor_reader.set_turret_encoder(make_encoder(6));
    sensor_reader.set_turret_hall(make_unique<DigitalInput>(2));

    sensor_reader.set_hood_encoder(make_encoder(4));
    sensor_reader.set_hood_index(make_unique<DigitalInput>(1));

    sensor_reader.set_shooter_encoder(make_encoder(2));

    sensor_reader.set_autonomous_mode(0, make_unique<DigitalInput>(9));
    sensor_reader.set_autonomous_mode(1, make_unique<DigitalInput>(8));

    sensor_reader.set_pwm_trigger(true);

    AddLoop(&sensor_reader_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop imu_event_loop(&config.message());
    auto imu_trigger = make_unique<DigitalInput>(3);
    ::frc971::wpilib::ADIS16448 imu(&imu_event_loop, SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.SetDummySPI(SPI::Port::kOnboardCS2);
    auto imu_reset = make_unique<DigitalOutput>(6);
    imu.set_reset(imu_reset.get());
    AddLoop(&imu_event_loop);

    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(7)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)), false);

    SuperstructureWriter superstructure_writer(&output_event_loop);
    superstructure_writer.set_intake_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)));
    superstructure_writer.set_intake_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    superstructure_writer.set_indexer_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    superstructure_writer.set_indexer_roller_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    superstructure_writer.set_turret_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(9)));
    superstructure_writer.set_hood_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    superstructure_writer.set_shooter_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(8)));

    superstructure_writer.set_gear_servo(
        ::std::unique_ptr<Servo>(new Servo(0)));

    superstructure_writer.set_red_light(
        ::std::unique_ptr<DigitalOutput>(new DigitalOutput(5)));
    superstructure_writer.set_green_light(
        ::std::unique_ptr<DigitalOutput>(new DigitalOutput(24)));
    superstructure_writer.set_blue_light(
        ::std::unique_ptr<DigitalOutput>(new DigitalOutput(25)));

    AddLoop(&output_event_loop);

    // Thread 6.
    ::aos::ShmEventLoop solenoid_writer_event_loop(&config.message());
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop);
    solenoid_writer.set_lights(solenoid_writer.pcm()->MakeSolenoid(0));
    solenoid_writer.set_rgb_light(solenoid_writer.pcm()->MakeSolenoid(1));
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2017

AOS_ROBOT_CLASS(::y2017::wpilib::WPILibRobot);
