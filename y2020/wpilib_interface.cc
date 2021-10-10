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
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/ADIS16470.h"
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
#include "gflags/gflags.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/shooter/shooter_tuning_readings_generated.h"
#include "y2020/control_loops/superstructure/superstructure_output_generated.h"
#include "y2020/control_loops/superstructure/superstructure_position_generated.h"

DEFINE_bool(shooter_tuning, true,
            "If true, reads from ball beambreak sensors and sends shooter "
            "tuning readings");

using ::aos::monotonic_clock;
using ::y2020::constants::Values;
namespace superstructure = ::y2020::control_loops::superstructure;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2020 {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

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

double turret_pot_translate(double voltage) {
  return voltage * Values::kTurretPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    std::max({Values::kMaxControlPanelEncoderPulsesPerSecond(),
              Values::kMaxFinisherEncoderPulsesPerSecond(),
              Values::kMaxAcceleratorEncoderPulsesPerSecond()});
static_assert(kMaxFastEncoderPulsesPerSecond <= 1000000.0,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    std::max({Values::kMaxDrivetrainEncoderPulsesPerSecond(),
              Values::kMaxHoodEncoderPulsesPerSecond(),
              Values::kMaxIntakeEncoderPulsesPerSecond(),
              Values::kMaxTurretEncoderPulsesPerSecond()});

static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000.0,
              "medium encoders are too fast");

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
                    "/drivetrain")),
        shooter_tuning_readings_sender_(
            event_loop->MakeSender<superstructure::shooter::TuningReadings>(
                "/superstructure")) {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    UpdateMediumEncoderFilterHz(kMaxMediumEncoderPulsesPerSecond);

    constants::InitValues();
  }

  // Hood
  void set_hood_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    hood_encoder_.set_encoder(::std::move(encoder));
  }

  void set_hood_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    hood_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_hood_single_turn_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    hood_encoder_.set_single_turn_absolute_pwm(::std::move(absolute_pwm));
  }

  // Intake

  void set_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    intake_joint_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    intake_joint_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  // Turret

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

  // Shooter
  void set_finisher_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    finisher_encoder_ = ::std::move(encoder);
  }
  void set_left_accelerator_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    left_accelerator_encoder_ = ::std::move(encoder);
  }

  void set_right_accelerator_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    right_accelerator_encoder_ = ::std::move(encoder);
  }

  // Auto mode switches.
  void set_autonomous_mode(int i, ::std::unique_ptr<frc::DigitalInput> sensor) {
    medium_encoder_filter_.Add(sensor.get());
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void set_imu(frc971::wpilib::ADIS16470 *imu) { imu_ = imu; }

  void set_ball_beambreak_inputs(::std::unique_ptr<frc::DigitalInput> sensor1,
                                 ::std::unique_ptr<frc::DigitalInput> sensor2) {
    ball_beambreak_inputs_[0] = ::std::move(sensor1);
    ball_beambreak_inputs_[1] = ::std::move(sensor2);
    ball_beambreak_reader_.set_input_one(ball_beambreak_inputs_[0].get());
    ball_beambreak_reader_.set_input_two(ball_beambreak_inputs_[1].get());
  }

  void Start() override {
    if (FLAGS_shooter_tuning) {
      AddToDMA(&ball_beambreak_reader_);
    }
  }

  void RunIteration() override {
    if (imu_ != nullptr) {
      imu_->DoReads();
    }

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
    const constants::Values &values = constants::GetValues();

    {
      auto builder = superstructure_position_sender_.MakeBuilder();

      // TODO(alex): check new absolute encoder api.
      // Hood
      frc971::AbsoluteAndAbsolutePositionT hood;
      CopyPosition(hood_encoder_, &hood,
                   Values::kHoodEncoderCountsPerRevolution(),
                   Values::kHoodEncoderRatio(),
                   Values::kHoodSingleTurnEncoderRatio(), false);
      flatbuffers::Offset<frc971::AbsoluteAndAbsolutePosition> hood_offset =
          frc971::AbsoluteAndAbsolutePosition::Pack(*builder.fbb(), &hood);

      // Intake
      frc971::AbsolutePositionT intake_joint;
      CopyPosition(intake_joint_encoder_, &intake_joint,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), false);
      flatbuffers::Offset<frc971::AbsolutePosition> intake_joint_offset =
          frc971::AbsolutePosition::Pack(*builder.fbb(), &intake_joint);

      // Turret
      frc971::PotAndAbsolutePositionT turret;
      CopyPosition(turret_encoder_, &turret,
                   Values::kTurretEncoderCountsPerRevolution(),
                   Values::kTurretEncoderRatio(), turret_pot_translate, true,
                   values.turret.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> turret_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &turret);

      // Shooter
      y2020::control_loops::superstructure::ShooterPositionT shooter;
      shooter.theta_finisher =
          encoder_translate(-finisher_encoder_->GetRaw(),
                            Values::kFinisherEncoderCountsPerRevolution(),
                            Values::kFinisherEncoderRatio());

      shooter.theta_accelerator_left =
          encoder_translate(-left_accelerator_encoder_->GetRaw(),
                            Values::kAcceleratorEncoderCountsPerRevolution(),
                            Values::kAcceleratorEncoderRatio());
      shooter.theta_accelerator_right =
          encoder_translate(right_accelerator_encoder_->GetRaw(),
                            Values::kAcceleratorEncoderCountsPerRevolution(),
                            Values::kAcceleratorEncoderRatio());
      flatbuffers::Offset<y2020::control_loops::superstructure::ShooterPosition>
          shooter_offset =
              y2020::control_loops::superstructure::ShooterPosition::Pack(
                  *builder.fbb(), &shooter);

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();
      position_builder.add_hood(hood_offset);
      position_builder.add_intake_joint(intake_joint_offset);
      position_builder.add_turret(turret_offset);
      position_builder.add_shooter(shooter_offset);

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

    if (FLAGS_shooter_tuning) {
      // Distance between beambreak sensors, in meters.
      constexpr double kDistanceBetweenBeambreaks = 0.4813;

      if (ball_beambreak_reader_.pulses_detected() > balls_detected_) {
        balls_detected_ = ball_beambreak_reader_.pulses_detected();

        auto builder = shooter_tuning_readings_sender_.MakeBuilder();
        auto shooter_tuning_readings_builder =
            builder.MakeBuilder<superstructure::shooter::TuningReadings>();
        shooter_tuning_readings_builder.add_velocity_ball(
            kDistanceBetweenBeambreaks / ball_beambreak_reader_.last_width());
        builder.Send(shooter_tuning_readings_builder.Finish());
      }
    }
  }

 private:
  ::aos::Sender<::frc971::autonomous::AutonomousMode> auto_mode_sender_;
  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
  ::aos::Sender<superstructure::shooter::TuningReadings>
      shooter_tuning_readings_sender_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer turret_encoder_;

  ::frc971::wpilib::AbsoluteAndAbsoluteEncoder hood_encoder_;

  ::frc971::wpilib::AbsoluteEncoder intake_joint_encoder_;

  ::std::unique_ptr<::frc::Encoder> finisher_encoder_,
      left_accelerator_encoder_, right_accelerator_encoder_;

  ::std::array<::std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  frc971::wpilib::ADIS16470 *imu_ = nullptr;

  // Used to interface with the two beam break sensors that the ball for tuning
  // shooter parameters has to pass through.
  // We will time how long it takes to pass between the two sensors to get its
  // velocity.
  std::array<std::unique_ptr<frc::DigitalInput>, 2> ball_beambreak_inputs_;
  frc971::wpilib::DMAPulseSeparationReader ball_beambreak_reader_;
  int balls_detected_ = 0;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  void set_hood_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hood_victor_ = ::std::move(t);
  }

  void set_intake_joint_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_joint_victor_ = ::std::move(t);
  }

  void set_intake_roller_falcon(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    intake_roller_falcon_ = ::std::move(t);
    intake_roller_falcon_->ConfigSupplyCurrentLimit(
        {true, Values::kIntakeRollerSupplyCurrentLimit(),
         Values::kIntakeRollerSupplyCurrentLimit(), 0});
    intake_roller_falcon_->ConfigStatorCurrentLimit(
        {true, Values::kIntakeRollerStatorCurrentLimit(),
         Values::kIntakeRollerStatorCurrentLimit(), 0});
  }

  void set_turret_victor(::std::unique_ptr<::frc::VictorSP> t) {
    turret_victor_ = ::std::move(t);
  }

  void set_feeder_falcon(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    feeder_falcon_ = ::std::move(t);
    {
      auto result = feeder_falcon_->ConfigSupplyCurrentLimit(
          {true, Values::kFeederSupplyCurrentLimit(),
           Values::kFeederSupplyCurrentLimit(), 0});
      if (result != ctre::phoenix::OKAY) {
        LOG(WARNING) << "Failed to configure feeder supply current limit: "
                     << result;
      }
    }
    {
      auto result = feeder_falcon_->ConfigStatorCurrentLimit(
          {true, Values::kFeederStatorCurrentLimit(),
           Values::kFeederStatorCurrentLimit(), 0});
      if (result != ctre::phoenix::OKAY) {
        LOG(WARNING) << "Failed to configure feeder stator current limit: "
                     << result;
      }
    }
  }

  void set_washing_machine_control_panel_victor(
      ::std::unique_ptr<::frc::VictorSP> t) {
    washing_machine_control_panel_victor_ = ::std::move(t);
  }

  void set_accelerator_left_falcon(::std::unique_ptr<::frc::TalonFX> t) {
    accelerator_left_falcon_ = ::std::move(t);
  }

  void set_accelerator_right_falcon(::std::unique_ptr<::frc::TalonFX> t) {
    accelerator_right_falcon_ = ::std::move(t);
  }

  void set_finisher_falcon0(::std::unique_ptr<::frc::TalonFX> t) {
    finisher_falcon0_ = ::std::move(t);
  }

  void set_finisher_falcon1(::std::unique_ptr<::frc::TalonFX> t) {
    finisher_falcon1_ = ::std::move(t);
  }

  void set_climber_falcon(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX> t) {
    climber_falcon_ = ::std::move(t);
    climber_falcon_->ConfigSupplyCurrentLimit(
        {true, Values::kClimberSupplyCurrentLimit(),
         Values::kClimberSupplyCurrentLimit(), 0});
  }

 private:
  void Write(const superstructure::Output &output) override {
    hood_victor_->SetSpeed(std::clamp(-output.hood_voltage(), -kMaxBringupPower,
                                      kMaxBringupPower) /
                           12.0);

    intake_joint_victor_->SetSpeed(std::clamp(-output.intake_joint_voltage(),
                                              -kMaxBringupPower,
                                              kMaxBringupPower) /
                                   12.0);

    intake_roller_falcon_->Set(
        ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
        std::clamp(-output.intake_roller_voltage(), -kMaxBringupPower,
                   kMaxBringupPower) /
            12.0);

    turret_victor_->SetSpeed(std::clamp(output.turret_voltage(),
                                        -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    feeder_falcon_->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                        std::clamp(output.feeder_voltage(), -kMaxBringupPower,
                                   kMaxBringupPower) /
                            12.0);
    if (washing_machine_control_panel_victor_) {
      washing_machine_control_panel_victor_->SetSpeed(
          std::clamp(-output.washing_machine_spinner_voltage(),
                     -kMaxBringupPower, kMaxBringupPower) /
          12.0);
    }

    accelerator_left_falcon_->SetSpeed(
        std::clamp(-output.accelerator_left_voltage(), -kMaxBringupPower,
                   kMaxBringupPower) /
        12.0);

    accelerator_right_falcon_->SetSpeed(
        std::clamp(output.accelerator_right_voltage(), -kMaxBringupPower,
                   kMaxBringupPower) /
        12.0);

    finisher_falcon1_->SetSpeed(std::clamp(output.finisher_voltage(),
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                                12.0);
    finisher_falcon0_->SetSpeed(std::clamp(-output.finisher_voltage(),
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                                12.0);

    if (climber_falcon_) {
      climber_falcon_->Set(
          ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
          std::clamp(-output.climber_voltage(), -kMaxBringupPower,
                     kMaxBringupPower) /
              12.0);
    }
  }

  void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    hood_victor_->SetDisabled();
    intake_joint_victor_->SetDisabled();
    turret_victor_->SetDisabled();
    if (washing_machine_control_panel_victor_) {
      washing_machine_control_panel_victor_->SetDisabled();
    }
    accelerator_left_falcon_->SetDisabled();
    accelerator_right_falcon_->SetDisabled();
    finisher_falcon0_->SetDisabled();
    finisher_falcon1_->SetDisabled();
    feeder_falcon_->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
    intake_roller_falcon_->Set(
        ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
    climber_falcon_->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
  }

  ::std::unique_ptr<::frc::VictorSP> hood_victor_, intake_joint_victor_,
      turret_victor_, washing_machine_control_panel_victor_;

  ::std::unique_ptr<::frc::TalonFX> accelerator_left_falcon_,
      accelerator_right_falcon_, finisher_falcon0_, finisher_falcon1_;

  ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonFX>
      intake_roller_falcon_, climber_falcon_, feeder_falcon_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(
      int index, frc::Encoder::EncodingType encodingType = frc::Encoder::k4X) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     encodingType);
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
    sensor_reader.set_pwm_trigger(true);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(0));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(1));
    // TODO: pin numbers
    sensor_reader.set_hood_encoder(
        make_unique<frc::Encoder>(22, 23, false, frc::Encoder::k4X));

    sensor_reader.set_hood_absolute_pwm(make_unique<frc::DigitalInput>(25));
    sensor_reader.set_hood_single_turn_absolute_pwm(
        make_unique<frc::DigitalInput>(24));

    sensor_reader.set_intake_encoder(make_encoder(5));
    sensor_reader.set_intake_absolute_pwm(make_unique<frc::DigitalInput>(1));

    sensor_reader.set_turret_encoder(make_encoder(2));
    sensor_reader.set_turret_absolute_pwm(make_unique<frc::DigitalInput>(0));
    sensor_reader.set_turret_potentiometer(make_unique<frc::AnalogInput>(0));

    sensor_reader.set_finisher_encoder(
        make_unique<frc::Encoder>(3, 2, false, frc::Encoder::k4X));
    sensor_reader.set_left_accelerator_encoder(make_encoder(4));
    sensor_reader.set_right_accelerator_encoder(make_encoder(3));

    if (FLAGS_shooter_tuning) {
      sensor_reader.set_ball_beambreak_inputs(
          make_unique<frc::DigitalInput>(6), make_unique<frc::DigitalInput>(7));
    }

    AddLoop(&sensor_reader_event_loop);

    // Note: If ADIS16470 is plugged in directly to the roboRIO SPI port without
    // the Spartan Board, then trigger is on 26, reset 27, and chip select is
    // CS0.
    // TODO(james): Double check whether the above is still accurate/useful with
    // the ADIS16448. No reason it shouldn't be.
    frc::SPI::Port spi_port = frc::SPI::Port::kOnboardCS1;
    std::unique_ptr<frc::DigitalInput> imu_trigger;
    std::unique_ptr<frc::DigitalOutput> imu_reset;
    if (::aos::network::GetTeamNumber() ==
        constants::Values::kCodingRobotTeamNumber) {
      imu_trigger = make_unique<frc::DigitalInput>(26);
      imu_reset = make_unique<frc::DigitalOutput>(27);
      spi_port = frc::SPI::Port::kOnboardCS0;
    } else {
      imu_trigger = make_unique<frc::DigitalInput>(9);
      imu_reset = make_unique<frc::DigitalOutput>(8);
    }
    ::aos::ShmEventLoop imu_event_loop(&config.message());
    std::unique_ptr<frc971::wpilib::ADIS16448> old_imu;
    std::unique_ptr<frc971::wpilib::ADIS16470> new_imu;
    std::unique_ptr<frc::SPI> imu_spi;
    if (::aos::network::GetTeamNumber() ==
        constants::Values::kCompTeamNumber) {
      old_imu = make_unique<frc971::wpilib::ADIS16448>(
          &imu_event_loop, spi_port, imu_trigger.get());
      old_imu->SetDummySPI(frc::SPI::Port::kOnboardCS2);
      old_imu->set_reset(imu_reset.get());
    } else {
      imu_spi = make_unique<frc::SPI>(spi_port);
      new_imu = make_unique<frc971::wpilib::ADIS16470>(
          &imu_event_loop, imu_spi.get(), imu_trigger.get(), imu_reset.get());
      sensor_reader.set_imu(new_imu.get());
    }
    AddLoop(&imu_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    output_event_loop.set_name("output_writer");
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), false);

    SuperstructureWriter superstructure_writer(&output_event_loop);
    superstructure_writer.set_hood_victor(make_unique<frc::VictorSP>(8));
    superstructure_writer.set_intake_joint_victor(
        make_unique<frc::VictorSP>(2));
    superstructure_writer.set_intake_roller_falcon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonFX>(0));
    superstructure_writer.set_turret_victor(make_unique<frc::VictorSP>(7));
    superstructure_writer.set_feeder_falcon(
        make_unique<ctre::phoenix::motorcontrol::can::TalonFX>(1));
    superstructure_writer.set_washing_machine_control_panel_victor(
        make_unique<frc::VictorSP>(6));
    superstructure_writer.set_accelerator_left_falcon(
        make_unique<::frc::TalonFX>(5));
    superstructure_writer.set_accelerator_right_falcon(
        make_unique<::frc::TalonFX>(4));
    superstructure_writer.set_finisher_falcon0(make_unique<::frc::TalonFX>(9));
    superstructure_writer.set_finisher_falcon1(make_unique<::frc::TalonFX>(3));
    // TODO: check port
    superstructure_writer.set_climber_falcon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonFX>(2));

    AddLoop(&output_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2020

AOS_ROBOT_CLASS(::y2020::wpilib::WPILibRobot);
