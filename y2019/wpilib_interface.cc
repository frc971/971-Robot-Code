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
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#include "ctre/phoenix/CANifier.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state.q.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/camera.q.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"
#include "y2019/jevois/spi.h"
#include "y2019/status_light.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::y2019::control_loops::superstructure::superstructure_queue;
using ::y2019::constants::Values;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
using aos::make_unique;

namespace y2019 {
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

double elevator_pot_translate(double voltage) {
  return voltage * Values::kElevatorPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double wrist_pot_translate(double voltage) {
  return voltage * Values::kWristPotRatio() * (5.0 /*turns*/ / 5.0 /*volts*/) *
         (2 * M_PI /*radians*/);
}

double stilts_pot_translate(double voltage) {
  return voltage * Values::kStiltsPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(Values::kMaxDrivetrainEncoderPulsesPerSecond(),
        Values::kMaxIntakeEncoderPulsesPerSecond());
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");
constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(Values::kMaxElevatorEncoderPulsesPerSecond(),
        Values::kMaxWristEncoderPulsesPerSecond());

static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop) {
    // Set to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateFastEncoderFilterHz(kMaxFastEncoderPulsesPerSecond);
    UpdateMediumEncoderFilterHz(kMaxMediumEncoderPulsesPerSecond);
  }

  // Elevator

  void set_elevator_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    elevator_encoder_.set_encoder(::std::move(encoder));
  }

  void set_elevator_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    elevator_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_elevator_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    elevator_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Intake

  void set_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  // Wrist

  void set_wrist_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
  }

  void set_wrist_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    wrist_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_wrist_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    wrist_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Stilts

  void set_stilts_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    stilts_encoder_.set_encoder(::std::move(encoder));
  }

  void set_stilts_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    stilts_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_stilts_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    stilts_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_platform_left_detect(
      ::std::unique_ptr<frc::DigitalInput> platform_left_detect) {
    platform_left_detect_ = ::std::move(platform_left_detect);
  }

  void set_platform_right_detect(
      ::std::unique_ptr<frc::DigitalInput> platform_right_detect) {
    platform_right_detect_ = ::std::move(platform_right_detect);
  }

  // Vacuum pressure sensor
  void set_vacuum_sensor(int port) {
    vacuum_sensor_ = make_unique<frc::AnalogInput>(port);
  }

  // Auto mode switches.
  void set_autonomous_mode(int i, ::std::unique_ptr<frc::DigitalInput> sensor) {
    autonomous_modes_.at(i) = ::std::move(sensor);
  }

  void RunIteration() override {
    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());

      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->right_speed = -drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }
    const auto values = constants::GetValues();

    {
      auto superstructure_message = superstructure_queue.position.MakeMessage();

      // Elevator
      CopyPosition(elevator_encoder_, &superstructure_message->elevator,
                   Values::kElevatorEncoderCountsPerRevolution(),
                   Values::kElevatorEncoderRatio(), elevator_pot_translate,
                   false, values.elevator.potentiometer_offset);
      // Intake
      CopyPosition(intake_encoder_, &superstructure_message->intake_joint,
                   Values::kIntakeEncoderCountsPerRevolution(),
                   Values::kIntakeEncoderRatio(), false);

      // Wrist
      CopyPosition(wrist_encoder_, &superstructure_message->wrist,
                   Values::kWristEncoderCountsPerRevolution(),
                   Values::kWristEncoderRatio(), wrist_pot_translate, false,
                   values.wrist.potentiometer_offset);

      // Stilts
      CopyPosition(stilts_encoder_, &superstructure_message->stilts,
                   Values::kStiltsEncoderCountsPerRevolution(),
                   Values::kStiltsEncoderRatio(), stilts_pot_translate, false,
                   values.stilts.potentiometer_offset);

      // Suction
      constexpr float kMinVoltage = 0.5;
      constexpr float kMaxVoltage = 2.1;
      superstructure_message->suction_pressure =
          (vacuum_sensor_->GetVoltage() - kMinVoltage) /
          (kMaxVoltage - kMinVoltage);

      superstructure_message->platform_left_detect =
          !platform_left_detect_->Get();
      superstructure_message->platform_right_detect =
          !platform_right_detect_->Get();

      superstructure_message.Send();
    }

    {
      auto auto_mode_message = ::frc971::autonomous::auto_mode.MakeMessage();
      auto_mode_message->mode = 0;
      for (size_t i = 0; i < autonomous_modes_.size(); ++i) {
        if (autonomous_modes_[i] && autonomous_modes_[i]->Get()) {
          auto_mode_message->mode |= 1 << i;
        }
      }
      LOG_STRUCT(DEBUG, "auto mode", *auto_mode_message);
      auto_mode_message.Send();
    }
  }

 private:
  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer elevator_encoder_,
      wrist_encoder_, stilts_encoder_;

  ::std::unique_ptr<frc::DigitalInput> platform_left_detect_;
  ::std::unique_ptr<frc::DigitalInput> platform_right_detect_;

  ::std::unique_ptr<frc::AnalogInput> vacuum_sensor_;

  ::std::array<::std::unique_ptr<frc::DigitalInput>, 2> autonomous_modes_;

  ::frc971::wpilib::AbsoluteEncoder intake_encoder_;
  // TODO(sabina): Add wrist and elevator hall effects.
};

class CameraReader {
 public:
  CameraReader() = default;
  CameraReader(const CameraReader &) = delete;
  CameraReader &operator=(const CameraReader &) = delete;

  void set_spi(frc::SPI *spi) {
    spi_ = spi;
    spi_->SetClockRate(1e6);
    spi_->SetChipSelectActiveHigh();
    spi_->SetClockActiveLow();
    spi_->SetSampleDataOnFalling();
    // It ignores you if you try changing this...
    spi_->SetMSBFirst();
  }

  void set_activate_usb(std::unique_ptr<frc::DigitalInput> activate_usb) {
    activate_usb_ = std::move(activate_usb);
  }

  void set_activate_passthrough(
      std::unique_ptr<frc::DigitalInput> activate_passthrough) {
    activate_passthrough_ = std::move(activate_passthrough);
  }

  void DoSpiTransaction() {
    using namespace frc971::jevois;
    RoborioToTeensy to_teensy{};
    to_teensy.realtime_now = aos::realtime_clock::now();
    camera_log.FetchLatest();
    if (activate_usb_ && !activate_usb_->Get()) {
      to_teensy.camera_command = CameraCommand::kUsb;
    } else if (activate_passthrough_ && !activate_passthrough_->Get()) {
      to_teensy.camera_command = CameraCommand::kCameraPassthrough;
    } else if (camera_log.get() && camera_log->log) {
      to_teensy.camera_command = CameraCommand::kLog;
    } else {
      to_teensy.camera_command = CameraCommand::kNormal;
    }

    std::array<char, spi_transfer_size() + 1> to_send{};
    {
      const auto to_send_data =
          gsl::make_span(to_send).last<spi_transfer_size()>();
      const auto encoded = SpiPackToTeensy(to_teensy);
      std::copy(encoded.begin(), encoded.end(), to_send_data.begin());
    }
    rx_clearer_.ClearRxFifo();
    // First, send recieve a dummy byte because the Teensy can't control what it
    // sends for the first byte.
    std::array<char, spi_transfer_size() + 1> to_receive;
    DoTransaction(to_send, to_receive);
    const auto unpacked = SpiUnpackToRoborio(
        gsl::make_span(to_receive).last(spi_transfer_size()));
    if (!unpacked) {
      LOG(INFO, "Decoding SPI data failed\n");
      return;
    }

    const auto now = aos::monotonic_clock::now();
    for (const auto &received : unpacked->frames) {
      auto to_send = control_loops::drivetrain::camera_frames.MakeMessage();
      // Add an extra 10ms delay to account for unmodeled delays that Austin
      // thinks exists.
      to_send->timestamp =
          std::chrono::nanoseconds(
              (now - received.age - ::std::chrono::milliseconds(10))
                  .time_since_epoch()).count();
      to_send->num_targets = received.targets.size();
      for (size_t i = 0; i < received.targets.size(); ++i) {
        to_send->targets[i].distance = received.targets[i].distance;
        to_send->targets[i].height = received.targets[i].height;
        to_send->targets[i].heading = received.targets[i].heading;
        to_send->targets[i].skew = received.targets[i].skew;
      }
      to_send->camera = received.camera_index;
      LOG_STRUCT(DEBUG, "camera_frames", *to_send);
      to_send.Send();
    }

    if (dummy_spi_) {
      uint8_t dummy_send, dummy_receive;
      dummy_spi_->Transaction(&dummy_send, &dummy_receive, 1);
    }
  }

  void DoTransaction(gsl::span<char> to_send, gsl::span<char> to_receive) {
    CHECK_EQ(to_send.size(), to_receive.size());
    const auto result = spi_->Transaction(
        reinterpret_cast<uint8_t *>(to_send.data()),
        reinterpret_cast<uint8_t *>(to_receive.data()), to_send.size());
    if (result == to_send.size()) {
      return;
    }
    if (result == -1) {
      LOG(INFO, "SPI::Transaction of %zd bytes failed\n", to_send.size());
      return;
    }
    LOG(FATAL, "SPI::Transaction returned something weird\n");
  }

  void SetDummySPI(frc::SPI::Port port) {
    dummy_spi_.reset(new frc::SPI(port));
    // Pick the same settings here in case the roboRIO decides to try something
    // stupid when switching.
    if (dummy_spi_) {
      dummy_spi_->SetClockRate(1e5);
      dummy_spi_->SetChipSelectActiveLow();
      dummy_spi_->SetClockActiveLow();
      dummy_spi_->SetSampleDataOnFalling();
      dummy_spi_->SetMSBFirst();
    }
  }

 private:
  frc::SPI *spi_ = nullptr;
  ::std::unique_ptr<frc::SPI> dummy_spi_;

  std::unique_ptr<frc::DigitalInput> activate_usb_;
  std::unique_ptr<frc::DigitalInput> activate_passthrough_;

  frc971::wpilib::SpiRxClearer rx_clearer_;
};

class SuperstructureWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler(event_loop),
        robot_state_fetcher_(
            event_loop->MakeFetcher<::aos::RobotState>(".aos.robot_state")) {}

  void set_elevator_victor(::std::unique_ptr<::frc::VictorSP> t) {
    elevator_victor_ = ::std::move(t);
  }

  void set_suction_victor(::std::unique_ptr<::frc::VictorSP> t) {
    suction_victor_ = ::std::move(t);
  }

  void set_intake_victor(::std::unique_ptr<::frc::VictorSP> t) {
    intake_victor_ = ::std::move(t);
  }

  void set_wrist_victor(::std::unique_ptr<::frc::VictorSP> t) {
    wrist_victor_ = ::std::move(t);
  }

  void set_stilts_victor(::std::unique_ptr<::frc::VictorSP> t) {
    stilts_victor_ = ::std::move(t);
  }

 private:
  void Read() override {
    ::y2019::control_loops::superstructure::superstructure_queue.output
        .FetchAnother();
  }

  void Write() override {
    auto &queue =
        ::y2019::control_loops::superstructure::superstructure_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    elevator_victor_->SetSpeed(::aos::Clip(queue->elevator_voltage,
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                               12.0);

    intake_victor_->SetSpeed(::aos::Clip(queue->intake_joint_voltage,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    wrist_victor_->SetSpeed(::aos::Clip(-queue->wrist_voltage,
                                        -kMaxBringupPower, kMaxBringupPower) /
                            12.0);

    stilts_victor_->SetSpeed(::aos::Clip(queue->stilts_voltage,
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);

    robot_state_fetcher_.Fetch();
    const double battery_voltage = robot_state_fetcher_.get()
                                       ? robot_state_fetcher_->voltage_battery
                                       : 12.0;

    // Throw a fast low pass filter on the battery voltage so we don't respond
    // too fast to noise.
    filtered_battery_voltage_ =
        0.5 * filtered_battery_voltage_ + 0.5 * battery_voltage;

    suction_victor_->SetSpeed(::aos::Clip(
        queue->pump_voltage / filtered_battery_voltage_, -1.0, 1.0));
  }

  void Stop() override {
    LOG(WARNING, "Superstructure output too old.\n");

    elevator_victor_->SetDisabled();
    intake_victor_->SetDisabled();
    wrist_victor_->SetDisabled();
    stilts_victor_->SetDisabled();
    suction_victor_->SetDisabled();
  }

  ::aos::Fetcher<::aos::RobotState> robot_state_fetcher_;

  ::std::unique_ptr<::frc::VictorSP> elevator_victor_, intake_victor_,
      wrist_victor_, stilts_victor_, suction_victor_;

  double filtered_battery_voltage_ = 12.0;
};

class SolenoidWriter {
 public:
  SolenoidWriter()
      : superstructure_(
            ".y2019.control_loops.superstructure.superstructure_queue.output") {
  }

  void set_big_suction_cup(int index0, int index1) {
    big_suction_cup0_ = pcm_.MakeSolenoid(index0);
    big_suction_cup1_ = pcm_.MakeSolenoid(index1);
  }
  void set_small_suction_cup(int index0, int index1) {
    small_suction_cup0_ = pcm_.MakeSolenoid(index0);
    small_suction_cup1_ = pcm_.MakeSolenoid(index1);
  }

  void set_intake_roller_talon(
      ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonSRX> t) {
    intake_rollers_talon_ = ::std::move(t);
    intake_rollers_talon_->ConfigContinuousCurrentLimit(10.0, 0);
    intake_rollers_talon_->EnableCurrentLimit(true);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(20),
                                        ::std::chrono::milliseconds(1));

    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      {
        superstructure_.FetchLatest();
        if (superstructure_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *superstructure_);

          big_suction_cup0_->Set(!superstructure_->intake_suction_bottom);
          big_suction_cup1_->Set(!superstructure_->intake_suction_bottom);
          small_suction_cup0_->Set(superstructure_->intake_suction_top);
          small_suction_cup1_->Set(superstructure_->intake_suction_top);

          intake_rollers_talon_->Set(
              ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
              ::aos::Clip(superstructure_->intake_roller_voltage,
                          -kMaxBringupPower, kMaxBringupPower) /
                  12.0);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;

        pcm_.Flush();
        to_log.read_solenoids = pcm_.GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }

      status_light.FetchLatest();
      // If we don't have a light request (or it's an old one), we are borked.
      // Flash the red light slowly.
      if (!status_light.get() ||
          status_light.Age() > chrono::milliseconds(100)) {
        StatusLight color;
        color.red = 0.0;
        color.green = 0.0;
        color.blue = 0.0;

        ++light_flash_;
        if (light_flash_ > 10) {
          color.red = 0.5;
        }

        if (light_flash_ > 20) {
          light_flash_ = 0;
        }

        LOG_STRUCT(DEBUG, "color", color);
        SetColor(color);
      } else {
        LOG_STRUCT(DEBUG, "color", *status_light);
        SetColor(*status_light);
      }
    }
  }

  void SetColor(const StatusLight &status_light) {
    // Save CAN bandwidth and CPU at the cost of RT.  Only change the light when
    // it actually changes.  This is pretty low priority anyways.
    static int time_since_last_send = 0;
    ++time_since_last_send;
    if (time_since_last_send > 10) {
      time_since_last_send = 0;
    }
    if (status_light.green != last_green_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(status_light.green,
                             ::ctre::phoenix::CANifier::LEDChannelA);
      last_green_ = status_light.green;
    }

    if (status_light.blue != last_blue_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(status_light.blue,
                             ::ctre::phoenix::CANifier::LEDChannelC);
      last_blue_ = status_light.blue;
    }

    if (status_light.red != last_red_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(status_light.red,
                             ::ctre::phoenix::CANifier::LEDChannelB);
      last_red_ = status_light.red;
    }
  }

  void Quit() { run_ = false; }

 private:
  ::frc971::wpilib::BufferedPcm pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> big_suction_cup0_,
      big_suction_cup1_, small_suction_cup0_, small_suction_cup1_;

  ::std::unique_ptr<::ctre::phoenix::motorcontrol::can::TalonSRX>
      intake_rollers_talon_;

  ::aos::Queue<
      ::y2019::control_loops::superstructure::SuperstructureQueue::Output>
      superstructure_;

  ::ctre::phoenix::CANifier canifier_{0};

  ::std::atomic<bool> run_{true};

  double last_red_ = -1.0;
  double last_green_ = -1.0;
  double last_blue_ = -1.0;

  int light_flash_ = 0;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
  }

  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::aos::ShmEventLoop event_loop;

    ::frc971::wpilib::JoystickSender joystick_sender(&event_loop);
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader(&event_loop);

    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    reader.set_elevator_encoder(make_encoder(4));
    reader.set_elevator_absolute_pwm(make_unique<frc::DigitalInput>(4));
    reader.set_elevator_potentiometer(make_unique<frc::AnalogInput>(4));

    reader.set_wrist_encoder(make_encoder(5));
    reader.set_wrist_absolute_pwm(make_unique<frc::DigitalInput>(5));
    reader.set_wrist_potentiometer(make_unique<frc::AnalogInput>(5));

    reader.set_intake_encoder(make_encoder(2));
    reader.set_intake_absolute_pwm(make_unique<frc::DigitalInput>(2));

    reader.set_stilts_encoder(make_encoder(3));
    reader.set_stilts_absolute_pwm(make_unique<frc::DigitalInput>(3));
    reader.set_stilts_potentiometer(make_unique<frc::AnalogInput>(3));

    reader.set_pwm_trigger(true);
    reader.set_vacuum_sensor(7);

    reader.set_platform_right_detect(make_unique<frc::DigitalInput>(6));
    reader.set_platform_left_detect(make_unique<frc::DigitalInput>(7));

    reader.set_autonomous_mode(0, make_unique<frc::DigitalInput>(22));
    reader.set_autonomous_mode(0, make_unique<frc::DigitalInput>(23));

    ::std::thread reader_thread(::std::ref(reader));

    CameraReader camera_reader;
    frc::SPI camera_spi(frc::SPI::Port::kOnboardCS3);
    camera_reader.set_spi(&camera_spi);
    camera_reader.SetDummySPI(frc::SPI::Port::kOnboardCS2);
    // Austin says 8, 9, 24, and 25 are good options to choose from for these.
    camera_reader.set_activate_usb(make_unique<frc::DigitalInput>(24));
    camera_reader.set_activate_passthrough(make_unique<frc::DigitalInput>(25));

    auto imu_trigger = make_unique<frc::DigitalInput>(0);
    ::frc971::wpilib::ADIS16448 imu(&event_loop, frc::SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.set_spi_idle_callback(
        [&camera_reader]() { camera_reader.DoSpiTransaction(); });
    auto imu_reset = make_unique<frc::DigitalOutput>(1);
    imu.set_reset(imu_reset.get());
    ::std::thread imu_thread(::std::ref(imu));

    // While as of 2/9/18 the drivetrain Victors are SPX, it appears as though
    // they are identical, as far as DrivetrainWriter is concerned, to the SP
    // variety so all the Victors are written as SPs.

    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)), false);
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    SuperstructureWriter superstructure_writer(&event_loop);
    superstructure_writer.set_elevator_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    // TODO(austin): Do the vacuum
    superstructure_writer.set_suction_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    superstructure_writer.set_intake_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)));
    superstructure_writer.set_wrist_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    superstructure_writer.set_stilts_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)));

    ::std::thread superstructure_writer_thread(
        ::std::ref(superstructure_writer));

    SolenoidWriter solenoid_writer;
    solenoid_writer.set_intake_roller_talon(
        make_unique<::ctre::phoenix::motorcontrol::can::TalonSRX>(10));
    solenoid_writer.set_big_suction_cup(0, 1);
    solenoid_writer.set_small_suction_cup(2, 3);

    ::std::thread solenoid_writer_thread(::std::ref(solenoid_writer));

    // Wait forever. Not much else to do...
    while (true) {
      const int r = select(0, nullptr, nullptr, nullptr, nullptr);
      if (r != 0) {
        PLOG(WARNING, "infinite select failed");
      } else {
        PLOG(WARNING, "infinite select succeeded??\n");
      }
    }

    LOG(ERROR, "Exiting WPILibRobot\n");

    solenoid_writer.Quit();
    solenoid_writer_thread.join();
    joystick_sender.Quit();
    joystick_thread.join();
    pdp_fetcher.Quit();
    pdp_fetcher_thread.join();
    reader.Quit();
    reader_thread.join();
    imu.Quit();
    imu_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();
    superstructure_writer.Quit();
    superstructure_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2019

AOS_ROBOT_CLASS(::y2019::wpilib::WPILibRobot);
