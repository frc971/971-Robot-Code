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

#include "absl/flags/flag.h"

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/TalonFX.h"
#undef ERROR

#include "ctre/phoenix/cci/Diagnostics_CCI.h"

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/can_configuration_generated.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_position_static.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/generic_can_writer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/swerve/swerve_drivetrain_writer.h"
#include "frc971/wpilib/swerve/swerve_module.h"
#include "frc971/wpilib/swerve/swerve_util.h"
#include "frc971/wpilib/talonfx.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2025/constants.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/control_loops/superstructure/led_indicator.h"
#include "y2025/control_loops/superstructure/superstructure_can_position_static.h"
#include "y2025/control_loops/superstructure/superstructure_output_generated.h"
#include "y2025/control_loops/superstructure/superstructure_position_generated.h"
#include "y2025/control_loops/superstructure/superstructure_position_static.h"

ABSL_FLAG(bool, ctre_diag_server, false,
          "If true, enable the diagnostics server for interacting with "
          "devices on the CAN bus using Phoenix Tuner");

using ::aos::monotonic_clock;
using ::frc971::CANConfiguration;
using ::frc971::wpilib::TalonFX;
using frc971::wpilib::swerve::DrivetrainWriter;
using frc971::wpilib::swerve::SwerveModule;
using ::y2025::constants::Values;
namespace superstructure = ::y2025::control_loops::superstructure;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2025::wpilib {
namespace {

// TODO: Replace these values once robot is built
constexpr double kMaxFastEncoderPulsesPerSecond =
    std::max({Values::kMaxDrivetrainEncoderPulsesPerSecond(),
              Values::kMaxElevatorEncoderPulsesPerSecond(),
              Values::kMaxPivotEncoderPulsesPerSecond(),
              Values::kMaxWristEncoderPulsesPerSecond()});

static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

double elevator_pot_translate(double voltage) {
  return -voltage * Values::kElevatorPotMetersPerVolt();
}
double pivot_pot_translate(double voltage) {
  return voltage * Values::kPivotPotRadiansPerVolt();
}
double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakePotRadiansPerVolt();
}

}  // namespace

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop,
               const Constants *robot_constants)
      : ::frc971::wpilib::SensorReader(event_loop),
        robot_constants_(robot_constants),
        superstructure_position_sender_(
            event_loop->MakeSender<superstructure::PositionStatic>(
                "/superstructure")),
        gyro_sender_(event_loop->MakeSender<::frc971::sensors::GyroReading>(
            "/drivetrain")),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<frc971::control_loops::swerve::PositionStatic>(
                    "/drivetrain")) {
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  };

  void Start() override {
    AddToDMA(&imu_yaw_rate_reader_);
    if (wrist_encoder_.encoder() != nullptr ||
        pivot_sensors_.encoder() != nullptr ||
        elevator_sensors_.encoder() != nullptr) {
      AddToDMA(&elevator_sensors_.reader());
      AddToDMA(&pivot_sensors_.reader());
      AddToDMA(&wrist_encoder_.reader());
    }
  }

  void set_yaw_rate_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_yaw_rate_input_ = ::std::move(sensor);
    imu_yaw_rate_reader_.set_input(imu_yaw_rate_input_.get());
  }

  void RunIteration() override {
    if (wrist_encoder_.encoder() != nullptr ||
        pivot_sensors_.encoder() != nullptr ||
        elevator_sensors_.encoder() != nullptr) {
      aos::Sender<superstructure::PositionStatic>::StaticBuilder builder =
          superstructure_position_sender_.MakeStaticBuilder();
      CopyPosition(elevator_sensors_, builder->add_elevator(),
                   Values::kElevatorEncoderCountsPerRevolution(),
                   Values::kElevatorEncoderMetersPerRadian(),
                   elevator_pot_translate, true,
                   robot_constants_->robot()
                       ->elevator_constants()
                       ->potentiometer_offset());
      CopyPosition(
          pivot_sensors_, builder->add_pivot(),
          Values::kPivotEncoderCountsPerRevolution(),
          Values::kPivotEncoderRatio(), pivot_pot_translate, false,
          robot_constants_->robot()->pivot_constants()->potentiometer_offset());
      CopyPosition(wrist_encoder_, builder->add_wrist(),
                   Values::kWristEncoderCountsPerRevolution(),
                   Values::kWristEncoderRatio(), true /* wrist flipped */);
      CopyPosition(*intake_pivot_potentiometer_, builder->add_ground_intake(),
                   intake_pot_translate, false,
                   robot_constants_->robot()->intake_pivot_pot_offset());

      builder.CheckOk(builder.Send());
    }

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

      builder.CheckOk(builder.Send(gyro_reading_builder.Finish()));
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
  void set_pivot(::std::unique_ptr<frc::Encoder> encoder,
                 ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                 ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    pivot_sensors_.set_encoder(::std::move(encoder));
    pivot_sensors_.set_absolute_pwm(::std::move(absolute_pwm));
    pivot_sensors_.set_potentiometer(::std::move(potentiometer));
  }

  void set_elevator(::std::unique_ptr<frc::Encoder> encoder,
                    ::std::unique_ptr<frc::DigitalInput> absolute_pwm,
                    ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    fast_encoder_filter_.Add(encoder.get());
    elevator_sensors_.set_encoder(::std::move(encoder));
    elevator_sensors_.set_absolute_pwm(::std::move(absolute_pwm));
    elevator_sensors_.set_potentiometer(::std::move(potentiometer));
  }

  void set_wrist(::std::unique_ptr<frc::Encoder> encoder,
                 ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
    wrist_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_intake_pivot(::std::unique_ptr<frc::AnalogInput> pot) {
    intake_pivot_potentiometer_ = ::std::move(pot);
  }

 private:
  const Constants *robot_constants_;

  frc971::wpilib::swerve::SwerveEncoders swerve_encoders_;
  aos::Sender<superstructure::PositionStatic> superstructure_position_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;

  std::unique_ptr<frc::DigitalInput> imu_yaw_rate_input_;

  std::unique_ptr<frc::AnalogInput> intake_pivot_potentiometer_;

  frc971::wpilib::DMAPulseWidthReader imu_yaw_rate_reader_;

  frc971::wpilib::DMAAbsoluteEncoderAndPotentiometer elevator_sensors_;
  frc971::wpilib::DMAAbsoluteEncoderAndPotentiometer pivot_sensors_;

  frc971::wpilib::DMAAbsoluteEncoder wrist_encoder_;

  aos::Sender<frc971::control_loops::swerve::PositionStatic>
      drivetrain_position_sender_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run971() {
    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("aos_config.json");

    frc971::constants::WaitForConstants<y2025::Constants>(&config.message());

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

    const CurrentLimits *current_limits =
        robot_constants->common()->current_limits();
    std::vector<ctre::phoenix6::BaseStatusSignal *> canivore_signals_registry;

    // Thread 3.
    ::aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    SensorReader sensor_reader(&sensor_reader_event_loop, robot_constants);
    sensor_reader.set_pwm_trigger(false);
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(25));
    sensor_reader.set_front_left_encoder(
        make_encoder(4), std::make_unique<frc::DigitalInput>(4));
    sensor_reader.set_front_right_encoder(
        make_encoder(5), std::make_unique<frc::DigitalInput>(5));
    sensor_reader.set_back_left_encoder(make_encoder(2),
                                        std::make_unique<frc::DigitalInput>(2));
    sensor_reader.set_back_right_encoder(
        make_encoder(3), std::make_unique<frc::DigitalInput>(3));
    sensor_reader.set_elevator(make_unique<frc::Encoder>(7, 6),
                               make_unique<frc::DigitalInput>(8),
                               make_unique<frc::AnalogInput>(0));
    sensor_reader.set_pivot(make_encoder(1), make_unique<frc::DigitalInput>(1),
                            make_unique<frc::AnalogInput>(1));
    sensor_reader.set_wrist(make_unique<frc::Encoder>(23, 22),
                            make_unique<frc::DigitalInput>(24));
    sensor_reader.set_intake_pivot(make_unique<frc::AnalogInput>(7));

    // TODO: Set the roborio ports

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    // Set up CAN.
    if (!absl::GetFlag(FLAGS_ctre_diag_server)) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    std::vector<ctre::phoenix6::BaseStatusSignal *> rio_signal_registry;

    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    ::aos::ShmEventLoop rio_sensor_reader_event_loop(&config.message());
    rio_sensor_reader_event_loop.set_name("RioSensorReader");

    aos::Sender<y2025::control_loops::superstructure::CANPositionStatic>
        superstructure_can_position_sender =
            can_sensor_reader_event_loop.MakeSender<
                y2025::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/canivore");

    std::vector<std::shared_ptr<TalonFX>> canivore_talons;
    std::vector<std::shared_ptr<TalonFX>> rio_talons;

    std::shared_ptr<TalonFX> elevator_one = std::make_shared<TalonFX>(
        5, false, "Drivetrain Bus", &canivore_signals_registry,
        current_limits->elevator_stator_current_limit(),
        current_limits->elevator_supply_current_limit());
    std::shared_ptr<TalonFX> elevator_two = std::make_shared<TalonFX>(
        9, false, "Drivetrain Bus", &canivore_signals_registry,
        current_limits->elevator_stator_current_limit(),
        current_limits->elevator_supply_current_limit());
    std::shared_ptr<TalonFX> end_effector = std::make_shared<TalonFX>(
        12, false, "rio", &rio_signal_registry,
        current_limits->end_effector_stator_current_limit(),
        current_limits->end_effector_supply_current_limit());
    // Set correct roborio ports for intake_pivot and rollers
    std::shared_ptr<TalonFX> intake_pivot = std::make_shared<TalonFX>(
        23, false, "rio", &rio_signal_registry,
        current_limits->intake_pivot_stator_current_limit(),
        current_limits->intake_pivot_supply_current_limit());
    std::shared_ptr<TalonFX> rollers = std::make_shared<TalonFX>(
        23, false, "rio", &rio_signal_registry,
        current_limits->roller_stator_current_limit(),
        current_limits->roller_supply_current_limit());
    canivore_talons.push_back(elevator_one);
    canivore_talons.push_back(elevator_two);
    rio_talons.push_back(end_effector);
    rio_talons.push_back(intake_pivot);
    rio_talons.push_back(rollers);

    std::shared_ptr<TalonFX> pivot =
        std::make_shared<TalonFX>(1, true, "rio", &rio_signal_registry,
                                  current_limits->pivot_stator_current_limit(),
                                  current_limits->pivot_supply_current_limit());
    rio_talons.push_back(pivot);
    aos::Sender<y2025::control_loops::superstructure::CANPositionStatic>
        superstructure_canivore_position_sender =
            rio_sensor_reader_event_loop.MakeSender<
                y2025::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/canivore");

    std::shared_ptr<TalonFX> climber_one = std::make_shared<TalonFX>(
        3, true, "rio", &rio_signal_registry,
        current_limits->climber_stator_current_limit(),
        current_limits->climber_supply_current_limit());
    std::shared_ptr<TalonFX> climber_two = std::make_shared<TalonFX>(
        10, true, "rio", &rio_signal_registry,
        current_limits->climber_stator_current_limit(),
        current_limits->climber_supply_current_limit());

    rio_talons.push_back(climber_one);
    rio_talons.push_back(climber_two);

    std::shared_ptr<TalonFX> wrist =
        std::make_shared<TalonFX>(13, false, "rio", &rio_signal_registry,
                                  current_limits->wrist_stator_current_limit(),
                                  current_limits->wrist_supply_current_limit());
    rio_talons.push_back(wrist);

    frc971::wpilib::CANSensorReader canivore_can_sensor_reader(
        &can_sensor_reader_event_loop, std::move(canivore_signals_registry),
        canivore_talons,
        [&elevator_one, &elevator_two,
         &superstructure_canivore_position_sender](
            ctre::phoenix::StatusCode status) {
          aos::Sender<y2025::control_loops::superstructure::CANPositionStatic>::
              StaticBuilder superstructure_canivore_builder =
                  superstructure_canivore_position_sender.MakeStaticBuilder();

          elevator_one->SerializePosition(
              superstructure_canivore_builder->add_elevator_one(),
              constants::Values::kElevatorOutputRatio);
          elevator_two->SerializePosition(
              superstructure_canivore_builder->add_elevator_two(),
              constants::Values::kElevatorOutputRatio);
          superstructure_canivore_builder->set_status(static_cast<int>(status));
          superstructure_canivore_builder.CheckOk(
              superstructure_canivore_builder.Send());
        },
        frc971::wpilib::CANSensorReader::SignalSync::kNoSync);

    aos::Sender<y2025::control_loops::superstructure::CANPositionStatic>
        superstructure_rio_position_sender =
            rio_sensor_reader_event_loop.MakeSender<
                y2025::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/rio");

    frc971::wpilib::CANSensorReader rio_can_sensor_reader(
        &rio_sensor_reader_event_loop, std::move(rio_signal_registry),
        rio_talons,
        [&pivot, &climber_one, &climber_two, &end_effector, &intake_pivot,
         &rollers, &wrist, &superstructure_rio_position_sender](
            ctre::phoenix::StatusCode status) {
          aos::Sender<y2025::control_loops::superstructure::CANPositionStatic>::
              StaticBuilder superstructure_rio_builder =
                  superstructure_rio_position_sender.MakeStaticBuilder();

          pivot->SerializePosition(superstructure_rio_builder->add_pivot(),
                                   constants::Values::kPivotOutputRatio);
          // TODO add real value
          end_effector->SerializePosition(
              superstructure_rio_builder->add_end_effector(), 1);
          climber_one->SerializePosition(
              superstructure_rio_builder->add_climber_one(),
              constants::Values::kClimberOutputRatio);
          climber_two->SerializePosition(
              superstructure_rio_builder->add_climber_two(),
              constants::Values::kClimberOutputRatio);

          wrist->SerializePosition(superstructure_rio_builder->add_wrist(),
                                   constants::Values::kWristOutputRatio);
          intake_pivot->SerializePosition(
              superstructure_rio_builder->add_ground_intake_pivot(),
              constants::Values::kIntakeOutputRatio);
          rollers->SerializePosition(
              superstructure_rio_builder->add_ground_intake_roller(), 1);

          superstructure_rio_builder->set_status(static_cast<int>(status));
          superstructure_rio_builder.CheckOk(superstructure_rio_builder.Send());
        },
        frc971::wpilib::CANSensorReader::SignalSync::kNoSync);

    AddLoop(&can_sensor_reader_event_loop);
    AddLoop(&rio_sensor_reader_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop can_output_event_loop(&config.message());
    can_output_event_loop.set_name("CANOutputWriter");

    frc971::wpilib::GenericCANWriter<control_loops::superstructure::Output>
        can_superstructure_writer(
            &can_output_event_loop,
            [](const control_loops::superstructure::Output &output,
               const std::map<std::string_view, std::shared_ptr<TalonFX>>
                   &talonfx_map) {
              talonfx_map.find("elevator_one")
                  ->second->WriteVoltage(output.elevator_voltage());
              talonfx_map.find("elevator_two")
                  ->second->WriteVoltage(output.elevator_voltage());
              talonfx_map.find("pivot")->second->WriteVoltage(
                  output.pivot_voltage());
              talonfx_map.find("climber_one")
                  ->second->WriteCurrent(output.climber_current(), 4.0);
              talonfx_map.find("climber_two")
                  ->second->WriteCurrent(-output.climber_current(), 4.0);
              talonfx_map.find("end_effector")
                  ->second->WriteVoltage(-output.end_effector_voltage());
              talonfx_map.find("wrist")->second->WriteVoltage(
                  output.wrist_voltage(), false);
              talonfx_map.find("intake_pivot")
                  ->second->WriteVoltage(output.ground_intake_pivot_voltage(),
                                         false);
              talonfx_map.find("rollers")->second->WriteVoltage(
                  output.ground_intake_roller_voltage(), false);
            });

    can_superstructure_writer.add_talonfx("elevator_one", elevator_one);
    can_superstructure_writer.add_talonfx("elevator_two", elevator_two);
    can_superstructure_writer.add_talonfx("end_effector", end_effector);
    can_superstructure_writer.add_talonfx("pivot", pivot);
    can_superstructure_writer.add_talonfx("climber_one", climber_one);
    can_superstructure_writer.add_talonfx("climber_two", climber_two);
    can_superstructure_writer.add_talonfx("wrist", wrist);
    can_superstructure_writer.add_talonfx("intake_pivot", intake_pivot);
    can_superstructure_writer.add_talonfx("rollers", rollers);

    can_output_event_loop.MakeWatcher(
        "/roborio", [&can_superstructure_writer](
                        const frc971::CANConfiguration &configuration) {
          can_superstructure_writer.HandleCANConfiguration(configuration);
        });

    AddLoop(&can_output_event_loop);

    // Set up LED Indicator with its own event loop thread
    /*::aos::ShmEventLoop led_indicator_event_loop(&config.message());
    led_indicator_event_loop.set_name("LedIndicator");
    control_loops::superstructure::LedIndicator led_indicator(
        &led_indicator_event_loop);
    AddLoop(&led_indicator_event_loop);*/

    RunLoops();
  }

  void Run9971() {
    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("aos_config.json");

    frc971::constants::WaitForConstants<y2025::Constants>(&config.message());

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

    LOG(INFO) << "Initialized Thread 1";

    // Thread 3.
    ::aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    SensorReader sensor_reader(&sensor_reader_event_loop, robot_constants);
    sensor_reader.set_pwm_trigger(false);
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(25));
    sensor_reader.set_front_left_encoder(
        make_encoder(0), std::make_unique<frc::DigitalInput>(0));
    sensor_reader.set_front_right_encoder(
        make_encoder(2), std::make_unique<frc::DigitalInput>(2));
    sensor_reader.set_back_left_encoder(make_encoder(1),
                                        std::make_unique<frc::DigitalInput>(1));
    sensor_reader.set_back_right_encoder(
        make_encoder(4), std::make_unique<frc::DigitalInput>(4));
    LOG(INFO) << "Initialized Thread 2";
    // TODO: Set the roborio ports

    AddLoop(&sensor_reader_event_loop);

    RunLoops();
  }

  void Run() override {
    const int team_number = aos::network::GetTeamNumber();
    switch (team_number) {
      case 971:
        LOG(INFO) << "Running 971";
        Run971();
        return;
      case 9971:
        LOG(INFO) << "Running 9971";
        Run9971();
        return;
    }
  }
};

}  // namespace y2025::wpilib

AOS_ROBOT_CLASS(::y2025::wpilib::WPILibRobot);
