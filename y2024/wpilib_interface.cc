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
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/talonfx.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2024/constants.h"
#include "y2024/control_loops/superstructure/superstructure_output_generated.h"
#include "y2024/control_loops/superstructure/superstructure_position_generated.h"

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

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

double drivetrain_velocity_translate(double in) {
  return (((1.0 / in) / Values::kDrivetrainCyclesPerRevolution()) *
          (2.0 * M_PI)) *
         Values::kDrivetrainEncoderRatio() *
         control_loops::drivetrain::kWheelRadius;
}

constexpr double kMaxFastEncoderPulsesPerSecond = std::max({
    Values::kMaxDrivetrainEncoderPulsesPerSecond(),
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
            event_loop->MakeSender<
                ::frc971::control_loops::drivetrain::PositionStatic>(
                "/drivetrain")),
        gyro_sender_(event_loop->MakeSender<::frc971::sensors::GyroReading>(
            "/drivetrain")){};
  void Start() override { AddToDMA(&imu_yaw_rate_reader_); }

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
      auto builder = superstructure_position_sender_.MakeBuilder();

      superstructure::Position::Builder position_builder =
          builder.MakeBuilder<superstructure::Position>();
      builder.CheckOk(builder.Send(position_builder.Finish()));
    }

    SendDrivetrainPosition(drivetrain_position_sender_.MakeStaticBuilder(),
                           drivetrain_velocity_translate,
                           constants::Values::DrivetrainEncoderToMeters, false,
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

 private:
  std::shared_ptr<const Values> values_;

  aos::Sender<frc971::autonomous::AutonomousMode> auto_mode_sender_;
  aos::Sender<superstructure::Position> superstructure_position_sender_;
  aos::Sender<frc971::control_loops::drivetrain::PositionStatic>
      drivetrain_position_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;

  std::array<std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  std::unique_ptr<frc::DigitalInput> imu_yaw_rate_input_;

  frc971::wpilib::DMAPulseWidthReader imu_yaw_rate_reader_;
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
    sensor_reader.set_drivetrain_left_encoder(make_encoder(1));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(0));
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(0));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    // Set up CAN.
    if (!FLAGS_ctre_diag_server) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry;

    std::shared_ptr<TalonFX> right_front = std::make_shared<TalonFX>(
        0, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> right_back = std::make_shared<TalonFX>(
        1, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> left_front = std::make_shared<TalonFX>(
        2, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> left_back = std::make_shared<TalonFX>(
        3, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());

    ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
        constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
    ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
        constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");

    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    // Creating list of talonfx for CANSensorReader
    std::vector<std::shared_ptr<TalonFX>> talonfxs;
    for (auto talonfx : {right_front, right_back, left_front, left_back}) {
      talonfxs.push_back(talonfx);
    }

    aos::Sender<CANPositionStatic> can_position_sender =
        can_sensor_reader_event_loop.MakeSender<CANPositionStatic>(
            "/drivetrain");

    frc971::wpilib::CANSensorReader can_sensor_reader(
        &can_sensor_reader_event_loop, std::move(signals_registry), talonfxs,
        [talonfxs, &can_position_sender](ctre::phoenix::StatusCode status) {
          aos::Sender<CANPositionStatic>::StaticBuilder builder =
              can_position_sender.MakeStaticBuilder();

          auto falcon_vector = builder->add_talonfxs();

          for (auto talonfx : talonfxs) {
            talonfx->SerializePosition(
                falcon_vector->emplace_back(),
                control_loops::drivetrain::kHighOutputRatio);
          }

          builder->set_timestamp(talonfxs.front()->GetTimestamp());
          builder->set_status(static_cast<int>(status));

          builder.CheckOk(builder.Send());
        });

    AddLoop(&can_sensor_reader_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop can_output_event_loop(&config.message());
    can_output_event_loop.set_name("CANOutputWriter");

    frc971::wpilib::CANDrivetrainWriter can_drivetrain_writer(
        &can_output_event_loop);

    can_drivetrain_writer.set_talonfxs({right_front, right_back},
                                       {left_front, left_back});

    can_output_event_loop.MakeWatcher(
        "/roborio", [&can_drivetrain_writer](
                        const frc971::CANConfiguration &configuration) {
          can_drivetrain_writer.HandleCANConfiguration(configuration);
        });

    AddLoop(&can_output_event_loop);

    // Thread 6

    RunLoops();
  }
};

}  // namespace y2024::wpilib

AOS_ROBOT_CLASS(::y2024::wpilib::WPILibRobot);
