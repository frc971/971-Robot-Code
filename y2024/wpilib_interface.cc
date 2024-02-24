#include <unistd.h>

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/TalonFX.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#undef ERROR

#include "ctre/phoenix/cci/Diagnostics_CCI.h"

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
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/can_configuration_generated.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/drivetrain_can_position_static.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/can_drivetrain_writer.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/generic_can_writer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/talonfx.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2024/constants.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/superstructure/superstructure_can_position_static.h"
#include "y2024/control_loops/superstructure/superstructure_output_generated.h"
#include "y2024/control_loops/superstructure/superstructure_position_generated.h"
#include "y2024/control_loops/superstructure/superstructure_position_static.h"

DEFINE_bool(ctre_diag_server, false,
            "If true, enable the diagnostics server for interacting with "
            "devices on the CAN bus using Phoenix Tuner");

using ::aos::monotonic_clock;
using ::frc971::CANConfiguration;
using ::frc971::control_loops::drivetrain::CANPositionStatic;
using ::frc971::wpilib::TalonFX;
using ::y2024::constants::Values;
namespace superstructure = ::y2024::control_loops::superstructure;
namespace drivetrain = ::y2024::control_loops::drivetrain;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2024::wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

double climber_pot_translate(double voltage) {
  return -1 * voltage * Values::kClimberPotMetersPerVolt();
}

double extend_pot_translate(double voltage) {
  return voltage * Values::kExtendPotMetersPerVolt();
}

double catapult_pot_translate(double voltage) {
  return voltage * Values::kCatapultPotRadiansPerVolt();
}

double turret_pot_translate(double voltage) {
  return voltage * Values::kTurretPotRadiansPerVolt();
}

double altitude_pot_translate(double voltage) {
  return -1 * voltage * Values::kAltitudePotRadiansPerVolt();
}

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

constexpr double kMaxFastEncoderPulsesPerSecond = std::max({
    Values::kMaxDrivetrainEncoderPulsesPerSecond(),
    Values::kMaxIntakePivotEncoderPulsesPerSecond(),
    Values::kMaxClimberEncoderPulsesPerSecond(),
    Values::kMaxExtendEncoderPulsesPerSecond(),
    Values::kMaxCatapultEncoderPulsesPerSecond(),
});

static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

}  // namespace

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop,
               const Constants *robot_constants)
      : ::frc971::wpilib::SensorReader(event_loop),
        robot_constants_(CHECK_NOTNULL(robot_constants)),
        auto_mode_sender_(
            event_loop->MakeSender<::frc971::autonomous::AutonomousMode>(
                "/autonomous")),
        superstructure_position_sender_(
            event_loop->MakeSender<superstructure::PositionStatic>(
                "/superstructure")),
        drivetrain_position_sender_(
            event_loop->MakeSender<
                ::frc971::control_loops::drivetrain::PositionStatic>(
                "/drivetrain")),
        gyro_sender_(event_loop->MakeSender<::frc971::sensors::GyroReading>(
            "/drivetrain")) {
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  };
  void Start() override {
    AddToDMA(&imu_yaw_rate_reader_);
    AddToDMA(&turret_encoder_.reader());
    AddToDMA(&altitude_encoder_.reader());
  }

  // Auto mode switches.
  void set_autonomous_mode(int i, ::std::unique_ptr<frc::DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void set_yaw_rate_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_yaw_rate_input_ = ::std::move(sensor);
    imu_yaw_rate_reader_.set_input(imu_yaw_rate_input_.get());
  }

  void RunIteration() override {
    {
      aos::Sender<superstructure::PositionStatic>::StaticBuilder builder =
          superstructure_position_sender_.MakeStaticBuilder();

      CopyPosition(intake_pivot_encoder_, builder->add_intake_pivot(),
                   Values::kIntakePivotEncoderCountsPerRevolution(),
                   Values::kIntakePivotEncoderRatio(), /* reversed: */ false);

      CopyPosition(climber_encoder_, builder->add_climber(),
                   Values::kClimberEncoderCountsPerRevolution(),
                   Values::kClimberEncoderMetersPerRadian(),
                   climber_pot_translate, false,
                   robot_constants_->robot()
                       ->climber_constants()
                       ->potentiometer_offset());

      CopyPosition(extend_encoder_, builder->add_extend(),
                   Values::kExtendEncoderCountsPerRevolution(),
                   Values::kExtendEncoderMetersPerRadian(),
                   extend_pot_translate, true,
                   robot_constants_->robot()
                       ->extend_constants()
                       ->potentiometer_offset());

      CopyPosition(catapult_encoder_, builder->add_catapult(),
                   Values::kCatapultEncoderCountsPerRevolution(),
                   Values::kCatapultEncoderRatio(), catapult_pot_translate,
                   true,
                   robot_constants_->robot()
                       ->catapult_constants()
                       ->potentiometer_offset());

      CopyPosition(turret_encoder_, builder->add_turret(),
                   Values::kTurretEncoderCountsPerRevolution(),
                   Values::kTurretEncoderRatio(), turret_pot_translate, true,
                   robot_constants_->robot()
                       ->turret_constants()
                       ->potentiometer_offset());

      CopyPosition(altitude_encoder_, builder->add_altitude(),
                   Values::kAltitudeEncoderCountsPerRevolution(),
                   Values::kAltitudeEncoderRatio(), altitude_pot_translate,
                   true,
                   robot_constants_->robot()
                       ->altitude_constants()
                       ->potentiometer_offset());

      builder->set_transfer_beambreak(transfer_beam_break_->Get());
      builder->set_catapult_beambreak(catapult_beam_break_->Get());
      builder.CheckOk(builder.Send());
    }

    SendDrivetrainPosition(drivetrain_position_sender_.MakeStaticBuilder(),
                           drivetrain_velocity_translate,
                           constants::Values::DrivetrainEncoderToMeters, true,
                           false);

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
      double velocity_duty_cycle =
          imu_yaw_rate_reader_.last_width() * kPWMFrequencyHz;

      constexpr double kDutyCycleScale =
          1 / (kScaledRangeHigh - kScaledRangeLow);
      // scale from 0.1 - 0.9 to 0 - 1
      double rescaled_velocity_duty_cycle =
          (velocity_duty_cycle - kScaledRangeLow) * kDutyCycleScale;

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

  void set_intake_pivot(::std::unique_ptr<frc::Encoder> encoder,
                        ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(encoder.get());
    intake_pivot_encoder_.set_encoder(::std::move(encoder));
    intake_pivot_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_transfer_beambreak(::std::unique_ptr<frc::DigitalInput> sensor) {
    transfer_beam_break_ = ::std::move(sensor);
  }

  void set_catapult_beambreak(::std::unique_ptr<frc::DigitalInput> sensor) {
    catapult_beam_break_ = ::std::move(sensor);
  }

  void set_climber(::std::unique_ptr<frc::Encoder> encoder,
                   ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                   ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    climber_encoder_.set_encoder(::std::move(encoder));
    climber_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
    climber_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_extend(::std::unique_ptr<frc::Encoder> encoder,
                  ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                  ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    extend_encoder_.set_encoder(::std::move(encoder));
    extend_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
    extend_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_catapult(::std::unique_ptr<frc::Encoder> encoder,
                    ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                    ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    catapult_encoder_.set_encoder(::std::move(encoder));
    catapult_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
    catapult_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_turret(::std::unique_ptr<frc::Encoder> encoder,
                  ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                  ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    turret_encoder_.set_encoder(::std::move(encoder));
    turret_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
    turret_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_altitude(::std::unique_ptr<frc::Encoder> encoder,
                    ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                    ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    altitude_encoder_.set_encoder(::std::move(encoder));
    altitude_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
    altitude_encoder_.set_potentiometer(::std::move(potentiometer));
  }

 private:
  const Constants *robot_constants_;

  aos::Sender<frc971::autonomous::AutonomousMode> auto_mode_sender_;
  aos::Sender<superstructure::PositionStatic> superstructure_position_sender_;
  aos::Sender<frc971::control_loops::drivetrain::PositionStatic>
      drivetrain_position_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;

  std::array<std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  std::unique_ptr<frc::DigitalInput> imu_yaw_rate_input_, transfer_beam_break_,
      catapult_beam_break_;

  frc971::wpilib::AbsoluteEncoder intake_pivot_encoder_;
  frc971::wpilib::AbsoluteEncoderAndPotentiometer climber_encoder_,
      catapult_encoder_, extend_encoder_;

  frc971::wpilib::DMAPulseWidthReader imu_yaw_rate_reader_;

  frc971::wpilib::DMAAbsoluteEncoderAndPotentiometer turret_encoder_,
      altitude_encoder_;
};

class SuperstructurePWMWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructurePWMWriter(aos::EventLoop *event_loop)
      : frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  void set_catapult_kraken_one(::std::unique_ptr<::frc::TalonFX> t) {
    catapult_kraken_one_ = ::std::move(t);
  }
  void set_catapult_kraken_two(::std::unique_ptr<::frc::TalonFX> t) {
    catapult_kraken_two_ = ::std::move(t);
  }

 private:
  void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");
    catapult_kraken_one_->SetDisabled();
    catapult_kraken_two_->SetDisabled();
  }

  void Write(const superstructure::Output &output) override {
    WritePwm(output.catapult_voltage(), catapult_kraken_one_.get());
    WritePwm(output.catapult_voltage(), catapult_kraken_two_.get());
  }

  template <typename T>
  static void WritePwm(const double voltage, T *motor) {
    motor->SetSpeed(std::clamp(voltage, -kMaxBringupPower, kMaxBringupPower) /
                    12.0);
  }
  ::std::unique_ptr<::frc::TalonFX> catapult_kraken_one_, catapult_kraken_two_;
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

    frc971::constants::WaitForConstants<y2024::Constants>(&config.message());

    ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());
    frc971::constants::ConstantsFetcher<Constants> constants_fetcher(
        &constant_fetcher_event_loop);
    const Constants *robot_constants = &constants_fetcher.constants();

    AddLoop(&constant_fetcher_event_loop);

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
    SensorReader sensor_reader(&sensor_reader_event_loop, robot_constants);
    sensor_reader.set_pwm_trigger(true);
    sensor_reader.set_drivetrain_left_encoder(
        std::make_unique<frc::Encoder>(6, 7));
    sensor_reader.set_drivetrain_right_encoder(
        std::make_unique<frc::Encoder>(8, 9));
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(25));
    sensor_reader.set_intake_pivot(make_encoder(3),
                                   make_unique<frc::DigitalInput>(3));
    sensor_reader.set_transfer_beambreak(make_unique<frc::DigitalInput>(23));
    sensor_reader.set_catapult_beambreak(make_unique<frc::DigitalInput>(24));

    sensor_reader.set_climber(make_encoder(5),
                              make_unique<frc::DigitalInput>(5),
                              make_unique<frc::AnalogInput>(5));
    sensor_reader.set_extend(make_encoder(4), make_unique<frc::DigitalInput>(4),
                             make_unique<frc::AnalogInput>(4));
    sensor_reader.set_catapult(make_encoder(0),
                               make_unique<frc::DigitalInput>(0),
                               make_unique<frc::AnalogInput>(0));
    sensor_reader.set_turret(make_encoder(2), make_unique<frc::DigitalInput>(2),
                             make_unique<frc::AnalogInput>(2));
    sensor_reader.set_altitude(make_encoder(1),
                               make_unique<frc::DigitalInput>(1),
                               make_unique<frc::AnalogInput>(1));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    // Set up CAN.
    if (!FLAGS_ctre_diag_server) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    std::vector<ctre::phoenix6::BaseStatusSignal *> canivore_signal_registry;
    std::vector<ctre::phoenix6::BaseStatusSignal *> rio_signal_registry;

    const CurrentLimits *current_limits =
        robot_constants->common()->current_limits();

    std::shared_ptr<TalonFX> right_front = std::make_shared<TalonFX>(
        2, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->drivetrain_supply_current_limit(),
        current_limits->drivetrain_stator_current_limit());
    std::shared_ptr<TalonFX> right_back = std::make_shared<TalonFX>(
        1, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->drivetrain_supply_current_limit(),
        current_limits->drivetrain_stator_current_limit());
    std::shared_ptr<TalonFX> left_front = std::make_shared<TalonFX>(
        4, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->drivetrain_supply_current_limit(),
        current_limits->drivetrain_stator_current_limit());
    std::shared_ptr<TalonFX> left_back = std::make_shared<TalonFX>(
        5, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->drivetrain_supply_current_limit(),
        current_limits->drivetrain_stator_current_limit());
    std::shared_ptr<TalonFX> intake_pivot = std::make_shared<TalonFX>(
        6, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->intake_pivot_stator_current_limit(),
        current_limits->intake_pivot_supply_current_limit());
    std::shared_ptr<TalonFX> altitude = std::make_shared<TalonFX>(
        9, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->altitude_stator_current_limit(),
        current_limits->altitude_supply_current_limit());
    std::shared_ptr<TalonFX> turret = std::make_shared<TalonFX>(
        3, false, "Drivetrain Bus", &canivore_signal_registry,
        current_limits->turret_stator_current_limit(),
        current_limits->turret_supply_current_limit());
    std::shared_ptr<TalonFX> intake_roller = std::make_shared<TalonFX>(
        8, false, "rio", &rio_signal_registry,
        current_limits->intake_roller_stator_current_limit(),
        current_limits->intake_roller_supply_current_limit());
    std::shared_ptr<TalonFX> retention_roller = std::make_shared<TalonFX>(
        10, true, "rio", &rio_signal_registry,
        current_limits->retention_roller_stator_current_limit(),
        current_limits->retention_roller_supply_current_limit());
    std::shared_ptr<TalonFX> transfer_roller = std::make_shared<TalonFX>(
        9, true, "rio", &rio_signal_registry,
        current_limits->transfer_roller_stator_current_limit(),
        current_limits->transfer_roller_supply_current_limit());
    std::shared_ptr<TalonFX> climber = std::make_shared<TalonFX>(
        7, false, "rio", &rio_signal_registry,
        current_limits->climber_stator_current_limit(),
        current_limits->climber_supply_current_limit());
    std::shared_ptr<TalonFX> extend = std::make_shared<TalonFX>(
        11, false, "rio", &rio_signal_registry,
        current_limits->extend_stator_current_limit(),
        current_limits->extend_supply_current_limit());
    std::shared_ptr<TalonFX> extend_roller = std::make_shared<TalonFX>(
        12, true, "rio", &rio_signal_registry,
        current_limits->extend_roller_stator_current_limit(),
        current_limits->extend_roller_supply_current_limit());

    ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
        constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
    ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
        constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");

    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    ::aos::ShmEventLoop rio_sensor_reader_event_loop(&config.message());
    rio_sensor_reader_event_loop.set_name("RioSensorReader");

    // Creating list of talonfx for CANSensorReader
    std::vector<std::shared_ptr<TalonFX>> drivetrain_talonfxs;
    std::vector<std::shared_ptr<TalonFX>> canivore_talonfxs;
    std::vector<std::shared_ptr<TalonFX>> rio_talonfxs;

    for (auto talonfx : {right_front, right_back, left_front, left_back}) {
      drivetrain_talonfxs.push_back(talonfx);
      canivore_talonfxs.push_back(talonfx);
    }

    for (auto talonfx : {intake_pivot, altitude, turret}) {
      canivore_talonfxs.push_back(talonfx);
    }

    for (auto talonfx : {intake_roller, transfer_roller, climber, extend,
                         extend_roller, retention_roller}) {
      rio_talonfxs.push_back(talonfx);
    }

    aos::Sender<frc971::control_loops::drivetrain::CANPositionStatic>
        drivetrain_can_position_sender =
            can_sensor_reader_event_loop.MakeSender<
                frc971::control_loops::drivetrain::CANPositionStatic>(
                "/drivetrain");

    aos::Sender<y2024::control_loops::superstructure::CANPositionStatic>
        superstructure_can_position_sender =
            can_sensor_reader_event_loop.MakeSender<
                y2024::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/canivore");

    frc971::wpilib::CANSensorReader canivore_can_sensor_reader(
        &can_sensor_reader_event_loop, std::move(canivore_signal_registry),
        canivore_talonfxs,
        [drivetrain_talonfxs, &intake_pivot, &altitude, &turret,
         &drivetrain_can_position_sender, &superstructure_can_position_sender](
            ctre::phoenix::StatusCode status) {
          aos::Sender<frc971::control_loops::drivetrain::CANPositionStatic>::
              StaticBuilder drivetrain_can_builder =
                  drivetrain_can_position_sender.MakeStaticBuilder();

          auto drivetrain_falcon_vector =
              CHECK_NOTNULL(drivetrain_can_builder->add_talonfxs());

          for (auto talonfx : drivetrain_talonfxs) {
            talonfx->SerializePosition(
                drivetrain_falcon_vector->emplace_back(),
                control_loops::drivetrain::kHighOutputRatio);
          }

          drivetrain_can_builder->set_timestamp(
              drivetrain_talonfxs.front()->GetTimestamp());
          drivetrain_can_builder->set_status(static_cast<int>(status));

          drivetrain_can_builder.CheckOk(drivetrain_can_builder.Send());

          aos::Sender<y2024::control_loops::superstructure::CANPositionStatic>::
              StaticBuilder superstructure_can_builder =
                  superstructure_can_position_sender.MakeStaticBuilder();

          intake_pivot->SerializePosition(
              superstructure_can_builder->add_intake_pivot(),
              control_loops::drivetrain::kHighOutputRatio);
          altitude->SerializePosition(
              superstructure_can_builder->add_altitude(),
              control_loops::drivetrain::kHighOutputRatio);
          turret->SerializePosition(
              superstructure_can_builder->add_turret(),
              control_loops::drivetrain::kHighOutputRatio);

          superstructure_can_builder->set_timestamp(
              intake_pivot->GetTimestamp());
          superstructure_can_builder->set_status(static_cast<int>(status));
          superstructure_can_builder.CheckOk(superstructure_can_builder.Send());
        });

    aos::Sender<y2024::control_loops::superstructure::CANPositionStatic>
        superstructure_rio_position_sender =
            rio_sensor_reader_event_loop.MakeSender<
                y2024::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/rio");

    frc971::wpilib::CANSensorReader rio_can_sensor_reader(
        &rio_sensor_reader_event_loop, std::move(rio_signal_registry),
        rio_talonfxs,
        [&intake_roller, &transfer_roller, &climber, &extend, &extend_roller,
         &retention_roller, &superstructure_rio_position_sender](
            ctre::phoenix::StatusCode status) {
          aos::Sender<y2024::control_loops::superstructure::CANPositionStatic>::
              StaticBuilder superstructure_can_builder =
                  superstructure_rio_position_sender.MakeStaticBuilder();

          intake_roller->SerializePosition(
              superstructure_can_builder->add_intake_roller(),
              constants::Values::kIntakeRollerOutputRatio);
          transfer_roller->SerializePosition(
              superstructure_can_builder->add_transfer_roller(),
              constants::Values::kIntakeRollerOutputRatio);
          climber->SerializePosition(superstructure_can_builder->add_climber(),
                                     superstructure::climber::kOutputRatio);
          extend->SerializePosition(superstructure_can_builder->add_extend(),
                                    superstructure::extend::kOutputRatio);
          extend_roller->SerializePosition(
              superstructure_can_builder->add_extend_roller(),
              constants::Values::kExtendRollerOutputRatio);
          retention_roller->SerializePosition(
              superstructure_can_builder->add_retention_roller(), 1.0);

          superstructure_can_builder->set_timestamp(
              intake_roller->GetTimestamp());
          superstructure_can_builder->set_status(static_cast<int>(status));
          superstructure_can_builder.CheckOk(superstructure_can_builder.Send());
        });

    AddLoop(&can_sensor_reader_event_loop);
    AddLoop(&rio_sensor_reader_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop can_output_event_loop(&config.message());
    can_output_event_loop.set_name("CANOutputWriter");

    frc971::wpilib::CANDrivetrainWriter can_drivetrain_writer(
        &can_output_event_loop);

    frc971::wpilib::GenericCANWriter<control_loops::superstructure::Output>
        can_superstructure_writer(
            &can_output_event_loop,
            [](const control_loops::superstructure::Output &output,
               const std::map<std::string_view, std::shared_ptr<TalonFX>>
                   &talonfx_map) {
              talonfx_map.find("intake_pivot")
                  ->second->WriteVoltage(output.intake_pivot_voltage());
              talonfx_map.find("intake_roller")
                  ->second->WriteVoltage(output.intake_roller_voltage());
              talonfx_map.find("transfer_roller")
                  ->second->WriteVoltage(output.transfer_roller_voltage());
              talonfx_map.find("climber")->second->WriteVoltage(
                  output.climber_voltage());
              talonfx_map.find("extend")->second->WriteVoltage(
                  output.extend_voltage());
              talonfx_map.find("extend_roller")
                  ->second->WriteVoltage(output.extend_roller_voltage());
              talonfx_map.find("altitude")
                  ->second->WriteVoltage(output.altitude_voltage());
              talonfx_map.find("turret")->second->WriteVoltage(
                  output.turret_voltage());
              talonfx_map.find("retention_roller")
                  ->second->WriteVoltage(output.retention_roller_voltage());
              if (output.has_retention_roller_stator_current_limit()) {
                talonfx_map.find("retention_roller")
                    ->second->set_stator_current_limit(
                        output.retention_roller_stator_current_limit());
              }
            });

    can_drivetrain_writer.set_talonfxs({right_front, right_back},
                                       {left_front, left_back});

    can_superstructure_writer.add_talonfx("intake_pivot", intake_pivot);
    can_superstructure_writer.add_talonfx("intake_roller", intake_roller);
    can_superstructure_writer.add_talonfx("transfer_roller", transfer_roller);
    can_superstructure_writer.add_talonfx("climber", climber);
    can_superstructure_writer.add_talonfx("extend", extend);
    can_superstructure_writer.add_talonfx("extend_roller", extend_roller);
    can_superstructure_writer.add_talonfx("altitude", altitude);
    can_superstructure_writer.add_talonfx("turret", turret);
    can_superstructure_writer.add_talonfx("retention_roller", retention_roller);

    can_output_event_loop.MakeWatcher(
        "/roborio", [&can_drivetrain_writer, &can_superstructure_writer](
                        const frc971::CANConfiguration &configuration) {
          can_drivetrain_writer.HandleCANConfiguration(configuration);
          can_superstructure_writer.HandleCANConfiguration(configuration);
        });

    AddLoop(&can_output_event_loop);

    ::aos::ShmEventLoop pwm_event_loop(&config.message());
    SuperstructurePWMWriter superstructure_pwm_writer(&pwm_event_loop);
    superstructure_pwm_writer.set_catapult_kraken_one(
        make_unique<frc::TalonFX>(0));
    superstructure_pwm_writer.set_catapult_kraken_two(
        make_unique<frc::TalonFX>(1));

    AddLoop(&pwm_event_loop);
    // Thread 6

    RunLoops();
  }
};

}  // namespace y2024::wpilib

AOS_ROBOT_CLASS(::y2024::wpilib::WPILibRobot);
