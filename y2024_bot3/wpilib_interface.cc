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
#include "frc971/can_configuration_generated.h"
#include "frc971/constants/constants_sender_lib.h"
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
#include "frc971/wpilib/talonfx.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2024_bot3/constants.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_can_position_static.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_output_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_position_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_position_static.h"

ABSL_FLAG(bool, ctre_diag_server, false,
          "If true, enable the diagnostics server for interacting with "
          "devices on the CAN bus using Phoenix Tuner");

using ::aos::monotonic_clock;
using ::frc971::CANConfiguration;
using ::frc971::wpilib::TalonFX;
using ::y2024_bot3::constants::Values;
namespace superstructure = ::y2024_bot3::control_loops::superstructure;
namespace chrono = ::std::chrono;
using std::make_unique;

namespace y2024_bot3::wpilib {
namespace {

constexpr double kMaxBringupPower = 12.0;

constexpr double kMaxFastEncoderPulsesPerSecond = std::max({
    1000000  // arbitrary number because we deleted all the stuff in this array
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
        robot_constants_(robot_constants),
        superstructure_position_sender_(
            event_loop->MakeSender<superstructure::PositionStatic>(
                "/superstructure")) {
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  };
  void Start() override { AddToDMA(&imu_yaw_rate_reader_); }

  void set_yaw_rate_input(::std::unique_ptr<frc::DigitalInput> sensor) {
    imu_yaw_rate_input_ = ::std::move(sensor);
    imu_yaw_rate_reader_.set_input(imu_yaw_rate_input_.get());
  }

  void RunIteration() override {
    aos::Sender<superstructure::PositionStatic>::StaticBuilder builder =
        superstructure_position_sender_.MakeStaticBuilder();

    builder.CheckOk(builder.Send());

    {
      auto builder = gyro_sender_.MakeBuilder();
      ::frc971::sensors::GyroReading::Builder gyro_reading_builder =
          builder.MakeBuilder<::frc971::sensors::GyroReading>();

      builder.CheckOk(builder.Send(gyro_reading_builder.Finish()));
    }
  }

 private:
  const Constants *robot_constants_;

  aos::Sender<superstructure::PositionStatic> superstructure_position_sender_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_sender_;

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
    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("aos_config.json");

    frc971::constants::WaitForConstants<y2024_bot3::Constants>(
        &config.message());

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
    sensor_reader.set_pwm_trigger(false);
    sensor_reader.set_yaw_rate_input(make_unique<frc::DigitalInput>(25));

    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    // Set up CAN.
    if (!absl::GetFlag(FLAGS_ctre_diag_server)) {
      c_Phoenix_Diagnostics_SetSecondsToStart(-1);
      c_Phoenix_Diagnostics_Dispose();
    }

    std::vector<ctre::phoenix6::BaseStatusSignal *> canivore_signal_registry;
    std::vector<ctre::phoenix6::BaseStatusSignal *> rio_signal_registry;

    ::aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
    can_sensor_reader_event_loop.set_name("CANSensorReader");

    ::aos::ShmEventLoop rio_sensor_reader_event_loop(&config.message());
    rio_sensor_reader_event_loop.set_name("RioSensorReader");

    // Creating list of talonfx for CANSensorReader
    std::vector<std::shared_ptr<TalonFX>> canivore_talonfxs;
    std::vector<std::shared_ptr<TalonFX>> rio_talonfxs;

    aos::Sender<y2024_bot3::control_loops::superstructure::CANPositionStatic>
        superstructure_can_position_sender =
            can_sensor_reader_event_loop.MakeSender<
                y2024_bot3::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/canivore");

    aos::Sender<y2024_bot3::control_loops::superstructure::CANPositionStatic>
        superstructure_rio_position_sender =
            rio_sensor_reader_event_loop.MakeSender<
                y2024_bot3::control_loops::superstructure::CANPositionStatic>(
                "/superstructure/rio");

    frc971::wpilib::CANSensorReader rio_can_sensor_reader(
        &rio_sensor_reader_event_loop, std::move(rio_signal_registry),
        rio_talonfxs,
        [&superstructure_rio_position_sender](
            ctre::phoenix::StatusCode status) {
          aos::Sender<
              y2024_bot3::control_loops::superstructure::CANPositionStatic>::
              StaticBuilder superstructure_can_builder =
                  superstructure_rio_position_sender.MakeStaticBuilder();

          superstructure_can_builder->set_status(static_cast<int>(status));
          superstructure_can_builder.CheckOk(superstructure_can_builder.Send());
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
              (void)output;
              (void)talonfx_map;
            });

    can_output_event_loop.MakeWatcher(
        "/roborio", [&can_superstructure_writer](
                        const frc971::CANConfiguration &configuration) {
          can_superstructure_writer.HandleCANConfiguration(configuration);
        });

    AddLoop(&can_output_event_loop);

    RunLoops();
  }
};

}  // namespace y2024_bot3::wpilib

AOS_ROBOT_CLASS(::y2024_bot3::wpilib::WPILibRobot);
