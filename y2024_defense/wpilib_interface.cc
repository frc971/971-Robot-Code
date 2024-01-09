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

#include "ctre/phoenix/cci/Diagnostics_CCI.h"
#include "ctre/phoenix6/TalonFX.hpp"

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
#include "frc971/control_loops/drivetrain/drivetrain_can_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/can_drivetrain_writer.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/talonfx.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2024_defense/constants.h"

DEFINE_bool(ctre_diag_server, false,
            "If true, enable the diagnostics server for interacting with "
            "devices on the CAN bus using Phoenix Tuner");

using ::aos::monotonic_clock;
using ::frc971::CANConfiguration;
using ::y2024_defense::constants::Values;

using frc971::control_loops::drivetrain::CANPosition;
using frc971::wpilib::TalonFX;

using std::make_unique;

namespace y2024_defense {
namespace wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

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
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                    "/drivetrain")),
        gyro_sender_(event_loop->MakeSender<::frc971::sensors::GyroReading>(
            "/drivetrain")) {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  }

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
    superstructure_reading_->Set(true);
    {
      {
        auto builder = drivetrain_position_sender_.MakeBuilder();
        frc971::control_loops::drivetrain::Position::Builder
            drivetrain_builder =
                builder
                    .MakeBuilder<frc971::control_loops::drivetrain::Position>();
        drivetrain_builder.add_left_encoder(
            constants::Values::DrivetrainEncoderToMeters(
                drivetrain_left_encoder_->GetRaw()));
        drivetrain_builder.add_left_speed(drivetrain_velocity_translate(
            drivetrain_left_encoder_->GetPeriod()));

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
        double velocity_duty_cycle =
            imu_yaw_rate_reader_.last_width() * kPWMFrequencyHz;

        constexpr double kDutyCycleScale =
            1 / (kScaledRangeHigh - kScaledRangeLow);
        // scale from 0.1 - 0.9 to 0 - 1
        double rescaled_velocity_duty_cycle =
            (velocity_duty_cycle - kScaledRangeLow) * kDutyCycleScale;

        if (!std::isnan(rescaled_velocity_duty_cycle)) {
          gyro_reading_builder.add_velocity(
              (rescaled_velocity_duty_cycle - 0.5) * kVelocityRadiansPerSecond);
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
  }

  std::shared_ptr<frc::DigitalOutput> superstructure_reading_;

  void set_superstructure_reading(
      std::shared_ptr<frc::DigitalOutput> superstructure_reading) {
    superstructure_reading_ = superstructure_reading;
  }

 private:
  std::shared_ptr<const Values> values_;

  aos::Sender<frc971::autonomous::AutonomousMode> auto_mode_sender_;
  aos::Sender<frc971::control_loops::drivetrain::Position>
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
    std::shared_ptr<frc::DigitalOutput> superstructure_reading =
        make_unique<frc::DigitalOutput>(25);

    sensor_reader.set_pwm_trigger(true);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(1));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(0));
    sensor_reader.set_superstructure_reading(superstructure_reading);
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(0));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry;

    std::shared_ptr<TalonFX> right_front = std::make_shared<TalonFX>(
        1, true, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> right_back = std::make_shared<TalonFX>(
        2, true, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> right_under = std::make_shared<TalonFX>(
        3, true, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> left_front = std::make_shared<TalonFX>(
        4, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> left_back = std::make_shared<TalonFX>(
        5, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<TalonFX> left_under = std::make_shared<TalonFX>(
        6, false, "Drivetrain Bus", &signals_registry,
        constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());

    // Setting up CAN.
    if (!FLAGS_ctre_diag_server) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    // Creating list of falcons for CANSensorReader
    std::vector<std::shared_ptr<TalonFX>> falcons;
    for (auto falcon : {right_front, right_back, right_under, left_front,
                        left_back, left_under}) {
      falcons.push_back(falcon);
    }

    ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
        constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
    ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
        constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");

    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    aos::Sender<CANPosition> can_position_sender =
        can_sensor_reader_event_loop.MakeSender<CANPosition>("/drivetrain");

    frc971::wpilib::CANSensorReader can_sensor_reader(
        &can_sensor_reader_event_loop, std::move(signals_registry), falcons,
        [falcons, &can_position_sender](ctre::phoenix::StatusCode status) {
          auto builder = can_position_sender.MakeBuilder();
          aos::SizedArray<
              flatbuffers::Offset<frc971::control_loops::CANTalonFX>, 6>
              flatbuffer_falcons;

          for (auto falcon : falcons) {
            falcon->SerializePosition(
                builder.fbb(), control_loops::drivetrain::kHighOutputRatio);
            std::optional<
                flatbuffers::Offset<frc971::control_loops::CANTalonFX>>
                falcon_offset = falcon->TakeOffset();

            CHECK(falcon_offset.has_value());

            flatbuffer_falcons.push_back(falcon_offset.value());
          }

          auto falcons_list =
              builder.fbb()
                  ->CreateVector<
                      flatbuffers::Offset<frc971::control_loops::CANTalonFX>>(
                      flatbuffer_falcons);

          frc971::control_loops::drivetrain::CANPosition::Builder
              can_position_builder = builder.MakeBuilder<
                  frc971::control_loops::drivetrain::CANPosition>();

          can_position_builder.add_talonfxs(falcons_list);
          can_position_builder.add_timestamp(falcons.front()->GetTimestamp());
          can_position_builder.add_status(static_cast<int>(status));

          builder.CheckOk(builder.Send(can_position_builder.Finish()));
        });

    AddLoop(&can_sensor_reader_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop can_drivetrain_writer_event_loop(&config.message());
    can_drivetrain_writer_event_loop.set_name("CANDrivetrainWriter");

    frc971::wpilib::CANDrivetrainWriter can_drivetrain_writer(
        &can_drivetrain_writer_event_loop);

    can_drivetrain_writer.set_talonfxs({right_front, right_back, right_under},
                                       {left_front, left_back, left_under});

    can_drivetrain_writer_event_loop.MakeWatcher(
        "/roborio", [&can_drivetrain_writer](
                        const frc971::CANConfiguration &configuration) {
          can_drivetrain_writer.HandleCANConfiguration(configuration);
        });

    AddLoop(&can_drivetrain_writer_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2024_defense

AOS_ROBOT_CLASS(::y2024_defense::wpilib::WPILibRobot);
