#include "ctre/phoenix/cci/Diagnostics_CCI.h"
#include "ctre/phoenix6/TalonFX.hpp"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/falcon.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/swerve/swerve_drivetrain_writer.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2023_bot4/constants.h"
#include "y2023_bot4/drivetrain_can_position_generated.h"
#include "y2023_bot4/drivetrain_position_generated.h"

DEFINE_bool(ctre_diag_server, false,
            "If true, enable the diagnostics server for interacting with "
            "devices on the CAN bus using Phoenix Tuner");

using frc971::wpilib::CANSensorReader;
using frc971::wpilib::Falcon;
using frc971::wpilib::swerve::DrivetrainWriter;
using frc971::wpilib::swerve::SwerveModule;

namespace drivetrain = frc971::control_loops::drivetrain;

namespace y2023_bot4 {
namespace wpilib {
namespace {

template <class T>
T value_or_exit(std::optional<T> optional) {
  CHECK(optional.has_value());
  return optional.value();
}

flatbuffers::Offset<frc971::AbsolutePosition> module_offset(
    frc971::AbsolutePosition::Builder builder,
    frc971::wpilib::AbsoluteEncoder *module) {
  builder.add_encoder(module->ReadRelativeEncoder());
  builder.add_absolute_encoder(module->ReadAbsoluteEncoder());

  return builder.Finish();
}

flatbuffers::Offset<SwerveModuleCANPosition> can_module_offset(
    SwerveModuleCANPosition::Builder builder,
    std::shared_ptr<SwerveModule> module) {
  std::optional<flatbuffers::Offset<control_loops::CANFalcon>> rotation_offset =
      module->rotation->TakeOffset();
  std::optional<flatbuffers::Offset<control_loops::CANFalcon>>
      translation_offset = module->translation->TakeOffset();

  CHECK(rotation_offset.has_value());
  CHECK(translation_offset.has_value());

  builder.add_rotation(rotation_offset.value());
  builder.add_translation(translation_offset.value());

  return builder.Finish();
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
               std::shared_ptr<const constants::Values> values)
      : ::frc971::wpilib::SensorReader(event_loop),
        values_(values),
        drivetrain_position_sender_(
            event_loop->MakeSender<AbsoluteDrivetrainPosition>("/drivetrain")) {
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  }

  void RunIteration() override {
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();

      auto front_left_offset =
          module_offset(builder.MakeBuilder<frc971::AbsolutePosition>(),
                        &front_left_encoder_);
      auto front_right_offset =
          module_offset(builder.MakeBuilder<frc971::AbsolutePosition>(),
                        &front_right_encoder_);
      auto back_left_offset = module_offset(
          builder.MakeBuilder<frc971::AbsolutePosition>(), &back_left_encoder_);
      auto back_right_offset =
          module_offset(builder.MakeBuilder<frc971::AbsolutePosition>(),
                        &back_right_encoder_);

      AbsoluteDrivetrainPosition::Builder drivetrain_position_builder =
          builder.MakeBuilder<AbsoluteDrivetrainPosition>();

      drivetrain_position_builder.add_follower_wheel_one_position(
          follower_wheel_one_encoder_->GetRaw());
      drivetrain_position_builder.add_follower_wheel_two_position(
          follower_wheel_two_encoder_->GetRaw());

      drivetrain_position_builder.add_front_left_position(front_left_offset);
      drivetrain_position_builder.add_front_right_position(front_right_offset);
      drivetrain_position_builder.add_back_left_position(back_left_offset);
      drivetrain_position_builder.add_back_right_position(back_right_offset);

      builder.CheckOk(builder.Send(drivetrain_position_builder.Finish()));
    }
  }

  void set_follower_wheel_one_encoder(std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    follower_wheel_one_encoder_ = std::move(encoder);
    follower_wheel_one_encoder_->SetMaxPeriod(0.005);
  }
  void set_follower_wheel_two_encoder(std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    follower_wheel_two_encoder_ = std::move(encoder);
    follower_wheel_two_encoder_->SetMaxPeriod(0.005);
  }

  void set_front_left_encoder(std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    front_left_encoder_.set_encoder(std::move(encoder));
  }
  void set_front_left_absolute_pwm(
      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    front_left_encoder_.set_absolute_pwm(std::move(absolute_pwm));
  }

  void set_front_right_encoder(std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    front_right_encoder_.set_encoder(std::move(encoder));
  }
  void set_front_right_absolute_pwm(
      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    front_right_encoder_.set_absolute_pwm(std::move(absolute_pwm));
  }

  void set_back_left_encoder(std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    back_left_encoder_.set_encoder(std::move(encoder));
  }
  void set_back_left_absolute_pwm(
      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    back_left_encoder_.set_absolute_pwm(std::move(absolute_pwm));
  }

  void set_back_right_encoder(std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    back_right_encoder_.set_encoder(std::move(encoder));
  }
  void set_back_right_absolute_pwm(
      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    back_right_encoder_.set_absolute_pwm(std::move(absolute_pwm));
  }

 private:
  std::shared_ptr<const constants::Values> values_;

  aos::Sender<AbsoluteDrivetrainPosition> drivetrain_position_sender_;

  std::unique_ptr<frc::Encoder> follower_wheel_one_encoder_,
      follower_wheel_two_encoder_;

  frc971::wpilib::AbsoluteEncoder front_left_encoder_, front_right_encoder_,
      back_left_encoder_, back_right_encoder_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return std::make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                          frc::Encoder::k4X);
  }
  void Run() override {
    std::shared_ptr<const constants::Values> values =
        std::make_shared<const constants::Values>(constants::MakeValues());

    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("aos_config.json");

    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry;
    std::vector<std::shared_ptr<Falcon>> falcons;

    // TODO(max): Change the CanBus names with TalonFX software.
    std::shared_ptr<SwerveModule> front_left = std::make_shared<SwerveModule>(
        frc971::wpilib::FalconParams{6, false},
        frc971::wpilib::FalconParams{5, false}, "Drivetrain Bus",
        &signals_registry, constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<SwerveModule> front_right = std::make_shared<SwerveModule>(
        frc971::wpilib::FalconParams{3, false},
        frc971::wpilib::FalconParams{4, false}, "Drivetrain Bus",
        &signals_registry, constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<SwerveModule> back_left = std::make_shared<SwerveModule>(
        frc971::wpilib::FalconParams{8, false},
        frc971::wpilib::FalconParams{7, false}, "Drivetrain Bus",
        &signals_registry, constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());
    std::shared_ptr<SwerveModule> back_right = std::make_shared<SwerveModule>(
        frc971::wpilib::FalconParams{2, false},
        frc971::wpilib::FalconParams{1, false}, "Drivetrain Bus",
        &signals_registry, constants::Values::kDrivetrainStatorCurrentLimit(),
        constants::Values::kDrivetrainSupplyCurrentLimit());

    // Thread 1
    aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    falcons.push_back(front_left->rotation);
    falcons.push_back(front_left->translation);

    falcons.push_back(front_right->rotation);
    falcons.push_back(front_right->translation);

    falcons.push_back(back_left->rotation);
    falcons.push_back(back_left->translation);

    falcons.push_back(back_right->rotation);
    falcons.push_back(back_right->translation);

    aos::Sender<AbsoluteCANPosition> can_position_sender =
        can_sensor_reader_event_loop.MakeSender<AbsoluteCANPosition>(
            "/drivetrain");

    CANSensorReader can_sensor_reader(
        &can_sensor_reader_event_loop, std::move(signals_registry), falcons,
        [this, falcons, front_left, front_right, back_left, back_right,
         &can_position_sender](ctre::phoenix::StatusCode status) {
          // TODO(max): use status properly in the flatbuffer.
          (void)status;

          auto builder = can_position_sender.MakeBuilder();

          for (auto falcon : falcons) {
            falcon->RefreshNontimesyncedSignals();
            falcon->SerializePosition(builder.fbb(), 1.0);
          }

          auto front_left_offset = can_module_offset(
              builder.MakeBuilder<SwerveModuleCANPosition>(), front_left);
          auto front_right_offset = can_module_offset(
              builder.MakeBuilder<SwerveModuleCANPosition>(), front_right);
          auto back_left_offset = can_module_offset(
              builder.MakeBuilder<SwerveModuleCANPosition>(), back_left);
          auto back_right_offset = can_module_offset(
              builder.MakeBuilder<SwerveModuleCANPosition>(), back_right);

          AbsoluteCANPosition::Builder can_position_builder =
              builder.MakeBuilder<AbsoluteCANPosition>();

          can_position_builder.add_front_left(front_left_offset);
          can_position_builder.add_front_right(front_right_offset);
          can_position_builder.add_back_left(back_left_offset);
          can_position_builder.add_back_right(back_right_offset);

          builder.CheckOk(builder.Send(can_position_builder.Finish()));
        });

    AddLoop(&can_sensor_reader_event_loop);

    // Thread 2
    // Setup CAN
    if (!FLAGS_ctre_diag_server) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
        constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
    ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
        constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");

    aos::ShmEventLoop drivetrain_writer_event_loop(&config.message());
    drivetrain_writer_event_loop.set_name("DrivetrainWriter");

    DrivetrainWriter drivetrain_writer(
        &drivetrain_writer_event_loop,
        constants::Values::kDrivetrainWriterPriority, 12);

    drivetrain_writer.set_falcons(front_left, front_right, back_left,
                                  back_right);

    AddLoop(&drivetrain_writer_event_loop);

    // Thread 3
    aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    sensor_reader_event_loop.set_name("SensorReader");
    SensorReader sensor_reader(&sensor_reader_event_loop, values);

    sensor_reader.set_follower_wheel_one_encoder(make_encoder(4));
    sensor_reader.set_follower_wheel_two_encoder(make_encoder(5));

    sensor_reader.set_front_left_encoder(make_encoder(1));
    sensor_reader.set_front_left_absolute_pwm(
        std::make_unique<frc::DigitalInput>(1));

    sensor_reader.set_front_right_encoder(make_encoder(0));
    sensor_reader.set_front_right_absolute_pwm(
        std::make_unique<frc::DigitalInput>(0));

    sensor_reader.set_back_left_encoder(make_encoder(2));
    sensor_reader.set_back_left_absolute_pwm(
        std::make_unique<frc::DigitalInput>(2));

    sensor_reader.set_back_right_encoder(make_encoder(3));
    sensor_reader.set_back_right_absolute_pwm(
        std::make_unique<frc::DigitalInput>(3));

    AddLoop(&sensor_reader_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2023_bot4

AOS_ROBOT_CLASS(::y2023_bot4::wpilib::WPILibRobot)
