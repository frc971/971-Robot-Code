#include "absl/flags/flag.h"
#include "ctre/phoenix/cci/Diagnostics_CCI.h"
#include "ctre/phoenix6/TalonFX.hpp"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_position_static.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/swerve/swerve_drivetrain_writer.h"
#include "frc971/wpilib/swerve/swerve_util.h"
#include "frc971/wpilib/talonfx.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2024_swerve/constants.h"
#include "y2024_swerve/constants/constants_generated.h"

ABSL_FLAG(bool, ctre_diag_server, false,
          "If true, enable the diagnostics server for interacting with "
          "devices on the CAN bus using Phoenix Tuner");

using frc971::wpilib::CANSensorReader;
using frc971::wpilib::TalonFX;
using frc971::wpilib::swerve::DrivetrainWriter;
using frc971::wpilib::swerve::SwerveModule;

namespace drivetrain = frc971::control_loops::drivetrain;

namespace y2024_swerve::wpilib {
namespace {

template <class T>
T value_or_exit(std::optional<T> optional) {
  CHECK(optional.has_value());
  return optional.value();
}

constexpr double kMaxFastEncoderPulsesPerSecond = std::max({
    constants::Values::kMaxDrivetrainEncoderPulsesPerSecond(),
});
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
}  // namespace

class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(aos::ShmEventLoop *event_loop,
               std::shared_ptr<const constants::Values> values,
               const Constants *robot_constants)
      : ::frc971::wpilib::SensorReader(event_loop),
        values_(values),
        robot_constants_(robot_constants),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<frc971::control_loops::swerve::PositionStatic>(
                    "/drivetrain")),
        gyro_sender_(event_loop->MakeSender<::frc971::sensors::GyroReading>(
            "/drivetrain")) {
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  }

  void Start() override { AddToDMA(&imu_yaw_rate_reader_); }

  void set_yaw_rate_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_yaw_rate_input_ = ::std::move(sensor);
    imu_yaw_rate_reader_.set_input(imu_yaw_rate_input_.get());
  }

  void RunIteration() override {
    {
      auto builder = drivetrain_position_sender_.MakeStaticBuilder();
      auto swerve_position_constants =
          robot_constants_->common()->swerve_positions_constants();

      swerve_encoders_.PopulatePosition(builder.get(),
                                        swerve_position_constants);

      builder.CheckOk(builder.Send());
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
        gyro_reading_builder.add_velocity((rescaled_velocity_duty_cycle - 0.5) *
                                          kVelocityRadiansPerSecond);
      }
      builder.CheckOk(builder.Send(gyro_reading_builder.Finish()));
    }
  }

  void set_front_left_encoder(std::unique_ptr<frc::Encoder> encoder,
                              std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(encoder.get());
    swerve_encoders_.set_front_left(std::move(encoder),
                                    std::move(absolute_pwm));
  }

  void set_front_right_encoder(
      std::unique_ptr<frc::Encoder> encoder,
      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(encoder.get());
    swerve_encoders_.set_front_right(std::move(encoder),
                                     std::move(absolute_pwm));
  }

  void set_back_left_encoder(std::unique_ptr<frc::Encoder> encoder,
                             std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(encoder.get());
    swerve_encoders_.set_back_left(std::move(encoder), std::move(absolute_pwm));
  }

  void set_back_right_encoder(std::unique_ptr<frc::Encoder> encoder,
                              std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(encoder.get());
    swerve_encoders_.set_back_right(std::move(encoder),
                                    std::move(absolute_pwm));
  }

 private:
  std::shared_ptr<const constants::Values> values_;

  const Constants *robot_constants_;

  aos::Sender<frc971::control_loops::swerve::PositionStatic>
      drivetrain_position_sender_;

  frc971::wpilib::swerve::SwerveEncoders swerve_encoders_;

  std::unique_ptr<frc::DigitalInput> imu_yaw_rate_input_;
  frc971::wpilib::DMAPulseWidthReader imu_yaw_rate_reader_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return std::make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                          frc::Encoder::k4X);
  }
  void Run() override {
    // Setup CAN
    if (!absl::GetFlag(FLAGS_ctre_diag_server)) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
        constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
    ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
        constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");
    std::shared_ptr<const constants::Values> values =
        std::make_shared<const constants::Values>(constants::MakeValues());

    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("aos_config.json");

    frc971::constants::WaitForConstants<y2024_swerve::Constants>(
        &config.message());

    ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());

    frc971::constants::ConstantsFetcher<Constants> constants_fetcher(
        &constant_fetcher_event_loop);

    const Constants *robot_constants = &constants_fetcher.constants();

    AddLoop(&constant_fetcher_event_loop);

    ::aos::ShmEventLoop joystick_sender_event_loop(&config.message());
    ::frc971::wpilib::JoystickSender joystick_sender(
        &joystick_sender_event_loop);
    AddLoop(&joystick_sender_event_loop);

    ::aos::ShmEventLoop pdp_fetcher_event_loop(&config.message());
    ::frc971::wpilib::PDPFetcher pdp_fetcher(&pdp_fetcher_event_loop);
    AddLoop(&pdp_fetcher_event_loop);

    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry;
    std::vector<std::shared_ptr<TalonFX>> falcons;

    // TODO(max): Change the CanBus names with TalonFX software.
    frc971::wpilib::swerve::SwerveModules modules{
        .front_left = std::make_shared<SwerveModule>(
            frc971::wpilib::TalonFXParams{6, true},
            frc971::wpilib::TalonFXParams{5, false}, "Drivetrain Bus",
            &signals_registry,
            constants::Values::kDrivetrainStatorCurrentLimit(),
            constants::Values::kDrivetrainSupplyCurrentLimit()),
        .front_right = std::make_shared<SwerveModule>(
            frc971::wpilib::TalonFXParams{3, true},
            frc971::wpilib::TalonFXParams{4, false}, "Drivetrain Bus",
            &signals_registry,
            constants::Values::kDrivetrainStatorCurrentLimit(),
            constants::Values::kDrivetrainSupplyCurrentLimit()),
        .back_left = std::make_shared<SwerveModule>(
            frc971::wpilib::TalonFXParams{7, true},
            frc971::wpilib::TalonFXParams{8, false}, "Drivetrain Bus",
            &signals_registry,
            constants::Values::kDrivetrainStatorCurrentLimit(),
            constants::Values::kDrivetrainSupplyCurrentLimit()),
        .back_right = std::make_shared<SwerveModule>(
            frc971::wpilib::TalonFXParams{2, true},
            frc971::wpilib::TalonFXParams{1, false}, "Drivetrain Bus",
            &signals_registry,
            constants::Values::kDrivetrainStatorCurrentLimit(),
            constants::Values::kDrivetrainSupplyCurrentLimit())};

    // Thread 1
    aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    modules.PopulateFalconsVector(&falcons);

    aos::Sender<frc971::control_loops::swerve::CanPositionStatic>
        can_position_sender =
            can_sensor_reader_event_loop
                .MakeSender<frc971::control_loops::swerve::CanPositionStatic>(
                    "/drivetrain");

    CANSensorReader can_sensor_reader(
        &can_sensor_reader_event_loop, std::move(signals_registry), falcons,
        [this, falcons, modules,
         &can_position_sender](ctre::phoenix::StatusCode status) {
          // TODO(max): use status properly in the flatbuffer.
          (void)status;

          aos::Sender<frc971::control_loops::swerve::CanPositionStatic>::
              StaticBuilder builder = can_position_sender.MakeStaticBuilder();

          for (auto falcon : falcons) {
            falcon->RefreshNontimesyncedSignals();
          }

          const frc971::wpilib::swerve::SwerveModule::ModuleGearRatios
              gear_ratios{
                  .rotation = constants::Values::kRotationModuleRatio(),
                  .translation = constants::Values::kTranslationModuleRatio()};
          modules.front_left->PopulateCanPosition(builder->add_front_left(),
                                                  gear_ratios);
          modules.front_right->PopulateCanPosition(builder->add_front_right(),
                                                   gear_ratios);
          modules.back_left->PopulateCanPosition(builder->add_back_left(),
                                                 gear_ratios);
          modules.back_right->PopulateCanPosition(builder->add_back_right(),
                                                  gear_ratios);

          builder.CheckOk(builder.Send());
        },
        CANSensorReader::SignalSync::kNoSync);

    AddLoop(&can_sensor_reader_event_loop);

    // Thread 2

    aos::ShmEventLoop drivetrain_writer_event_loop(&config.message());
    drivetrain_writer_event_loop.set_name("DrivetrainWriter");

    DrivetrainWriter drivetrain_writer(
        &drivetrain_writer_event_loop,
        constants::Values::kDrivetrainWriterPriority, 12);

    drivetrain_writer.set_talonfxs(modules);

    AddLoop(&drivetrain_writer_event_loop);

    // Thread 3
    aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    sensor_reader_event_loop.set_name("SensorReader");
    SensorReader sensor_reader(&sensor_reader_event_loop, values,
                               robot_constants);

    sensor_reader.set_front_left_encoder(
        make_encoder(3), std::make_unique<frc::DigitalInput>(3));

    sensor_reader.set_front_right_encoder(
        make_encoder(1), std::make_unique<frc::DigitalInput>(1));

    sensor_reader.set_back_left_encoder(make_encoder(4),
                                        std::make_unique<frc::DigitalInput>(4));

    sensor_reader.set_back_right_encoder(
        make_encoder(0), std::make_unique<frc::DigitalInput>(0));

    sensor_reader.set_yaw_rate_input(std::make_unique<frc::DigitalInput>(25));

    AddLoop(&sensor_reader_event_loop);

    RunLoops();
  }
};

}  // namespace y2024_swerve::wpilib

AOS_ROBOT_CLASS(::y2024_swerve::wpilib::WPILibRobot)
