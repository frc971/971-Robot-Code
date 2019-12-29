#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Servo.h"
#include "frc971/wpilib/ahal/VictorSP.h"
#include "ctre/phoenix/CANifier.h"
#undef ERROR

#include "aos/commonmath.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/make_unique.h"
#include "aos/time/time.h"
#include "aos/util/compiler_memory_barrier.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/wpilib/ADIS16448.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/superstructure_output_generated.h"
#include "y2018/control_loops/superstructure/superstructure_position_generated.h"
#include "y2018/status_light_generated.h"
#include "y2018/vision/vision_generated.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using aos::make_unique;
using ::aos::monotonic_clock;
using ::y2018::constants::Values;
namespace chrono = ::std::chrono;
namespace superstructure = ::y2018::control_loops::superstructure;

namespace y2018 {
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

double proximal_pot_translate(double voltage) {
  return -voltage * Values::kProximalPotRatio() *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double distal_pot_translate(double voltage) {
  return voltage * Values::kDistalPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double intake_pot_translate(double voltage) {
  return voltage * Values::kIntakeMotorPotRatio() *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double intake_spring_translate(double voltage) {
  return voltage * Values::kIntakeSpringRatio() * (2 * M_PI /*radians*/) /
         (5.0 /*volts*/);
}

// TODO() figure out differnce between max and min voltages on shifter pots.
// Returns value from 0.0 to 1.0, with 0.0 being close to low gear so it can be
// passed drectly into the drivetrain position queue.
double drivetrain_shifter_pot_translate(double voltage) {
  return (voltage - Values::kDrivetrainShifterPotMinVoltage()) /
         (Values::kDrivetrainShifterPotMaxVoltage() -
          Values::kDrivetrainShifterPotMinVoltage());
}

constexpr double kMaxFastEncoderPulsesPerSecond =
    max(Values::kMaxDrivetrainEncoderPulsesPerSecond(),
        Values::kMaxIntakeMotorEncoderPulsesPerSecond());
static_assert(kMaxFastEncoderPulsesPerSecond <= 1300000,
              "fast encoders are too fast");

constexpr double kMaxMediumEncoderPulsesPerSecond =
    max(Values::kMaxProximalEncoderPulsesPerSecond(),
        Values::kMaxDistalEncoderPulsesPerSecond());
static_assert(kMaxMediumEncoderPulsesPerSecond <= 400000,
              "medium encoders are too fast");

// Class to send position messages with sensor readings to our loops.
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
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
  }

  void set_left_drivetrain_shifter_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    left_drivetrain_shifter_ = ::std::move(potentiometer);
  }

  void set_right_drivetrain_shifter_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    right_drivetrain_shifter_ = ::std::move(potentiometer);
  }

  // Proximal joint.
  void set_proximal_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    proximal_encoder_.set_encoder(::std::move(encoder));
  }

  void set_proximal_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    proximal_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_proximal_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    proximal_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Distal joint.
  void set_distal_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    distal_encoder_.set_encoder(::std::move(encoder));
  }

  void set_distal_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(absolute_pwm.get());
    distal_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_distal_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    distal_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  // Left intake side.
  void set_left_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    left_intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_left_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(absolute_pwm.get());
    left_intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_left_intake_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    left_intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_left_intake_spring_angle(
      ::std::unique_ptr<frc::AnalogInput> encoder) {
    left_intake_spring_angle_ = ::std::move(encoder);
  }

  void set_left_intake_cube_detector(
      ::std::unique_ptr<frc::DigitalInput> input) {
    left_intake_cube_detector_ = ::std::move(input);
  }

  // Right intake side.
  void set_right_intake_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    fast_encoder_filter_.Add(encoder.get());
    right_intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_right_intake_absolute_pwm(
      ::std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    fast_encoder_filter_.Add(absolute_pwm.get());
    right_intake_encoder_.set_absolute_pwm(::std::move(absolute_pwm));
  }

  void set_right_intake_potentiometer(
      ::std::unique_ptr<frc::AnalogInput> potentiometer) {
    right_intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_right_intake_spring_angle(
      ::std::unique_ptr<frc::AnalogInput> encoder) {
    right_intake_spring_angle_ = ::std::move(encoder);
  }

  void set_right_intake_cube_detector(
      ::std::unique_ptr<frc::DigitalInput> input) {
    right_intake_cube_detector_ = ::std::move(input);
  }

  void set_claw_beambreak(::std::unique_ptr<frc::DigitalInput> input) {
    claw_beambreak_ = ::std::move(input);
  }

  void set_box_back_beambreak(::std::unique_ptr<frc::DigitalInput> input) {
    box_back_beambreak_ = ::std::move(input);
  }

  void set_lidar_lite_input(::std::unique_ptr<frc::DigitalInput> lidar_lite_input) {
    lidar_lite_input_ = ::std::move(lidar_lite_input);
    lidar_lite_.set_input(lidar_lite_input_.get());
  }

  void Start() { AddToDMA(&lidar_lite_); }

  void RunIteration() {
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();
      frc971::control_loops::drivetrain::Position::Builder drivetrain_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();

      drivetrain_builder.add_left_encoder(
          drivetrain_translate(drivetrain_left_encoder_->GetRaw()));
      drivetrain_builder.add_left_speed (
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));
      drivetrain_builder.add_left_shifter_position (
          drivetrain_shifter_pot_translate(
              left_drivetrain_shifter_->GetVoltage()));

      drivetrain_builder.add_right_encoder (
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw()));
      drivetrain_builder.add_right_speed (
          -drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod()));
      drivetrain_builder.add_right_shifter_position (
          drivetrain_shifter_pot_translate(
              right_drivetrain_shifter_->GetVoltage()));

      builder.Send(drivetrain_builder.Finish());
    }
  }

  void RunDmaIteration() {
    const auto values = constants::GetValues();

    {
      auto builder =
          superstructure_position_sender_.MakeBuilder();

      // Proximal arm
      frc971::PotAndAbsolutePositionT arm_proximal;
      CopyPosition(proximal_encoder_, &arm_proximal,
                   Values::kProximalEncoderCountsPerRevolution(),
                   Values::kProximalEncoderRatio(), proximal_pot_translate,
                   true, values.arm_proximal.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> arm_proximal_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &arm_proximal);

      // Distal arm
      frc971::PotAndAbsolutePositionT arm_distal;
      CopyPosition(distal_encoder_, &arm_distal,
                   Values::kDistalEncoderCountsPerRevolution(),
                   Values::kDistalEncoderRatio(), distal_pot_translate, true,
                   values.arm_distal.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition> arm_distal_offset =
          frc971::PotAndAbsolutePosition::Pack(*builder.fbb(), &arm_distal);

      superstructure::ArmPosition::Builder arm_position_builder =
          builder.MakeBuilder<superstructure::ArmPosition>();
      arm_position_builder.add_proximal(arm_proximal_offset);
      arm_position_builder.add_distal(arm_distal_offset);

      flatbuffers::Offset<superstructure::ArmPosition> arm_position_offset =
          arm_position_builder.Finish();

      // Left intake
      frc971::PotAndAbsolutePositionT left_intake_motor_position;
      CopyPosition(left_intake_encoder_,
                   &left_intake_motor_position,
                   Values::kIntakeMotorEncoderCountsPerRevolution(),
                   Values::kIntakeMotorEncoderRatio(), intake_pot_translate,
                   false, values.left_intake.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition>
          left_intake_motor_position_offset =
              frc971::PotAndAbsolutePosition::Pack(*builder.fbb(),
                                                   &left_intake_motor_position);

      // Right intake
      frc971::PotAndAbsolutePositionT right_intake_motor_position;
      CopyPosition(right_intake_encoder_,
                   &right_intake_motor_position,
                   Values::kIntakeMotorEncoderCountsPerRevolution(),
                   Values::kIntakeMotorEncoderRatio(), intake_pot_translate,
                   true, values.right_intake.potentiometer_offset);
      flatbuffers::Offset<frc971::PotAndAbsolutePosition>
          right_intake_motor_position_offset =
              frc971::PotAndAbsolutePosition::Pack(*builder.fbb(),
                                                   &right_intake_motor_position);

      superstructure::IntakeElasticSensors::Builder
          left_intake_sensors_builder =
              builder.MakeBuilder<superstructure::IntakeElasticSensors>();

      left_intake_sensors_builder.add_motor_position(
          left_intake_motor_position_offset);
      left_intake_sensors_builder.add_spring_angle(
          intake_spring_translate(left_intake_spring_angle_->GetVoltage()) +
          values.left_intake.spring_offset);
      left_intake_sensors_builder.add_beam_break(
          !left_intake_cube_detector_->Get());

      flatbuffers::Offset<superstructure::IntakeElasticSensors>
          left_intake_offset = left_intake_sensors_builder.Finish();

      superstructure::IntakeElasticSensors::Builder
          right_intake_sensors_builder =
              builder.MakeBuilder<superstructure::IntakeElasticSensors>();

      right_intake_sensors_builder.add_motor_position(
          right_intake_motor_position_offset);
      right_intake_sensors_builder.add_spring_angle(
          -intake_spring_translate(right_intake_spring_angle_->GetVoltage()) +
          values.right_intake.spring_offset);
      right_intake_sensors_builder.add_beam_break(
          !right_intake_cube_detector_->Get());

      flatbuffers::Offset<control_loops::superstructure::IntakeElasticSensors>
          right_intake_offset = right_intake_sensors_builder.Finish();

      superstructure::Position::Builder superstructure_builder =
          builder.MakeBuilder<superstructure::Position>();

      superstructure_builder.add_left_intake(left_intake_offset);
      superstructure_builder.add_right_intake(right_intake_offset);
      superstructure_builder.add_arm(arm_position_offset);

      superstructure_builder.add_claw_beambreak_triggered(
          !claw_beambreak_->Get());
      superstructure_builder.add_box_back_beambreak_triggered(
          !box_back_beambreak_->Get());

      superstructure_builder.add_box_distance(lidar_lite_.last_width() /
                                              0.00001 / 100.0 / 2);

      builder.Send(superstructure_builder.Finish());
    }
  }

 private:
  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;

  ::std::unique_ptr<frc::AnalogInput> left_drivetrain_shifter_,
      right_drivetrain_shifter_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer proximal_encoder_,
      distal_encoder_;

  ::frc971::wpilib::AbsoluteEncoderAndPotentiometer left_intake_encoder_,
      right_intake_encoder_;

  ::std::unique_ptr<frc::AnalogInput> left_intake_spring_angle_,
      right_intake_spring_angle_;
  ::std::unique_ptr<frc::DigitalInput> left_intake_cube_detector_,
      right_intake_cube_detector_;

  ::std::unique_ptr<frc::DigitalInput> claw_beambreak_;
  ::std::unique_ptr<frc::DigitalInput> box_back_beambreak_;

  ::std::unique_ptr<frc::DigitalInput> lidar_lite_input_;
  ::frc971::wpilib::DMAPulseWidthReader lidar_lite_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(::aos::ShmEventLoop *event_loop,
                 ::frc971::wpilib::BufferedPcm *pcm)
      : event_loop_(event_loop),
        drivetrain_fetcher_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::drivetrain::Output>(
                    "/drivetrain")),
        superstructure_fetcher_(
            event_loop->MakeFetcher<superstructure::Output>("/superstructure")),
        status_light_fetcher_(
            event_loop->MakeFetcher<::y2018::StatusLight>("/superstructure")),
        vision_status_fetcher_(
            event_loop->MakeFetcher<::y2018::vision::VisionStatus>("/vision")),
        pneumatics_to_log_sender_(
            event_loop->MakeSender<::frc971::wpilib::PneumaticsToLog>("/aos")),
        pcm_(pcm) {
    event_loop->set_name("Solenoids");
    event_loop_->SetRuntimeRealtimePriority(27);

    int32_t status = 0;
    HAL_CompressorHandle compressor_ = HAL_InitializeCompressor(0, &status);
    if (status != 0) {
      AOS_LOG(ERROR, "Compressor status is nonzero, %d\n",
              static_cast<int>(status));
    }
    HAL_SetCompressorClosedLoopControl(compressor_, true, &status);
    if (status != 0) {
      AOS_LOG(ERROR, "Compressor status is nonzero, %d\n",
              static_cast<int>(status));
    }

    event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                               ::std::chrono::milliseconds(20),
                               ::std::chrono::milliseconds(1));
  }

  // left drive
  // right drive
  //
  // claw
  // arm brakes
  // hook release
  // fork release
  void set_left_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    left_drivetrain_shifter_ = ::std::move(s);
  }
  void set_right_drivetrain_shifter(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    right_drivetrain_shifter_ = ::std::move(s);
  }

  void set_claw(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    claw_ = ::std::move(s);
  }

  void set_arm_brakes(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    arm_brakes_ = ::std::move(s);
  }

  void set_hook(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    hook_ = ::std::move(s);
  }

  void set_forks(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    forks_ = ::std::move(s);
  }

  void Loop(const int iterations) {
    if (iterations != 1) {
      AOS_LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    {
      drivetrain_fetcher_.Fetch();
      if (drivetrain_fetcher_.get()) {
        left_drivetrain_shifter_->Set(!drivetrain_fetcher_->left_high());
        right_drivetrain_shifter_->Set(!drivetrain_fetcher_->right_high());
      }
    }

    {
      superstructure_fetcher_.Fetch();
      if (superstructure_fetcher_.get()) {
        claw_->Set(!superstructure_fetcher_->claw_grabbed());
        arm_brakes_->Set(superstructure_fetcher_->release_arm_brake());
        hook_->Set(superstructure_fetcher_->hook_release());
        forks_->Set(superstructure_fetcher_->forks_release());
      }
    }

    {
      auto builder = pneumatics_to_log_sender_.MakeBuilder();

      ::frc971::wpilib::PneumaticsToLog::Builder to_log_builder =
          builder.MakeBuilder<frc971::wpilib::PneumaticsToLog>();

      pcm_->Flush();
      to_log_builder.add_read_solenoids(pcm_->GetAll());
      builder.Send(to_log_builder.Finish());
    }

    monotonic_clock::time_point monotonic_now = event_loop_->monotonic_now();
    status_light_fetcher_.Fetch();
    // If we don't have a light request (or it's an old one), we are borked.
    // Flash the red light slowly.
    StatusLightT color;
    if (!status_light_fetcher_.get() ||
        monotonic_now > status_light_fetcher_.context().monotonic_event_time +
                            chrono::milliseconds(100)) {
      color.red = 0.0;
      color.green = 0.0;
      color.blue = 0.0;

      vision_status_fetcher_.Fetch();
      ++light_flash_;
      if (light_flash_ > 10) {
        color.red = 0.5;
      } else if (!vision_status_fetcher_.get() ||
                 monotonic_now >
                     vision_status_fetcher_.context().monotonic_event_time +
                         chrono::seconds(1)) {
        color.red = 0.5;
        color.green = 0.5;
      }

      if (light_flash_ > 20) {
        light_flash_ = 0;
      }
    } else {
      status_light_fetcher_->UnPackTo(&color);
    }
    SetColor(color);
  }

  void SetColor(const StatusLightT &status_light) {
    // Save CAN bandwidth and CPU at the cost of RT.  Only change the light when
    // it actually changes.  This is pretty low priority anyways.
    static int time_since_last_send = 0;
    ++time_since_last_send;
    if (time_since_last_send > 10) {
      time_since_last_send = 0;
    }
    if (status_light.green != last_green_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(1.0 - status_light.green,
                             ::ctre::phoenix::CANifier::LEDChannelB);
      last_green_ = status_light.green;
    }

    if (status_light.blue != last_blue_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(1.0 - status_light.blue,
                             ::ctre::phoenix::CANifier::LEDChannelA);
      last_blue_ = status_light.blue;
    }

    if (status_light.red != last_red_ || time_since_last_send == 0) {
      canifier_.SetLEDOutput(1.0 - status_light.red,
                             ::ctre::phoenix::CANifier::LEDChannelC);
      last_red_ = status_light.red;
    }
  }

  void Quit() { run_ = false; }

 private:
  ::aos::EventLoop *event_loop_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output>
      drivetrain_fetcher_;
  ::aos::Fetcher<superstructure::Output> superstructure_fetcher_;
  ::aos::Fetcher<::y2018::StatusLight> status_light_fetcher_;
  ::aos::Fetcher<::y2018::vision::VisionStatus> vision_status_fetcher_;

  aos::Sender<::frc971::wpilib::PneumaticsToLog> pneumatics_to_log_sender_;

  ::frc971::wpilib::BufferedPcm *pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid>
      left_drivetrain_shifter_, right_drivetrain_shifter_, claw_, arm_brakes_,
      hook_, forks_;

  HAL_CompressorHandle compressor_;

  ::ctre::phoenix::CANifier canifier_{0};

  ::std::atomic<bool> run_{true};

  double last_red_ = -1.0;
  double last_green_ = -1.0;
  double last_blue_ = -1.0;

  int light_flash_ = 0;
};

class SuperstructureWriter
    : public ::frc971::wpilib::LoopOutputHandler<superstructure::Output> {
 public:
  SuperstructureWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<superstructure::Output>(
            event_loop, "/superstructure") {}

  void set_proximal_victor(::std::unique_ptr<::frc::VictorSP> t) {
    proximal_victor_ = ::std::move(t);
  }
  void set_distal_victor(::std::unique_ptr<::frc::VictorSP> t) {
    distal_victor_ = ::std::move(t);
  }

  void set_hanger_victor(::std::unique_ptr<::frc::VictorSP> t) {
    hanger_victor_ = ::std::move(t);
  }

  void set_left_intake_elastic_victor(::std::unique_ptr<::frc::VictorSP> t) {
    left_intake_elastic_victor_ = ::std::move(t);
  }
  void set_right_intake_elastic_victor(::std::unique_ptr<::frc::VictorSP> t) {
    right_intake_elastic_victor_ = ::std::move(t);
  }

  void set_left_intake_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    left_intake_rollers_victor_ = ::std::move(t);
  }

  void set_right_intake_rollers_victor(::std::unique_ptr<::frc::VictorSP> t) {
    right_intake_rollers_victor_ = ::std::move(t);
  }

 private:
  virtual void Write(const superstructure::Output &output) override {
    left_intake_elastic_victor_->SetSpeed(
        ::aos::Clip(-output.left_intake()->voltage_elastic(), -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    right_intake_elastic_victor_->SetSpeed(
        ::aos::Clip(output.right_intake()->voltage_elastic(), -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    left_intake_rollers_victor_->SetSpeed(
        ::aos::Clip(-output.left_intake()->voltage_rollers(), -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    right_intake_rollers_victor_->SetSpeed(
        ::aos::Clip(output.right_intake()->voltage_rollers(), -kMaxBringupPower,
                    kMaxBringupPower) /
        12.0);

    proximal_victor_->SetSpeed(::aos::Clip(-output.voltage_proximal(),
                                           -kMaxBringupPower,
                                           kMaxBringupPower) /
                               12.0);

    distal_victor_->SetSpeed(::aos::Clip(output.voltage_distal(),
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
    hanger_victor_->SetSpeed(::aos::Clip(-output.voltage_winch(),
                                         -kMaxBringupPower, kMaxBringupPower) /
                             12.0);
  }

  virtual void Stop() override {
    AOS_LOG(WARNING, "Superstructure output too old.\n");

    left_intake_rollers_victor_->SetDisabled();
    right_intake_rollers_victor_->SetDisabled();
    left_intake_elastic_victor_->SetDisabled();
    right_intake_elastic_victor_->SetDisabled();

    proximal_victor_->SetDisabled();
    distal_victor_->SetDisabled();
    hanger_victor_->SetDisabled();
  }

  ::std::unique_ptr<::frc::VictorSP> left_intake_rollers_victor_,
      right_intake_rollers_victor_, left_intake_elastic_victor_,
      right_intake_elastic_victor_, proximal_victor_, distal_victor_,
      hanger_victor_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<frc::Encoder> make_encoder(int index) {
    return make_unique<frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                     frc::Encoder::k4X);
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
    sensor_reader.set_left_drivetrain_shifter_potentiometer(
        make_unique<frc::AnalogInput>(6));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(1));
    sensor_reader.set_right_drivetrain_shifter_potentiometer(
        make_unique<frc::AnalogInput>(7));

    sensor_reader.set_proximal_encoder(make_encoder(4));
    sensor_reader.set_proximal_absolute_pwm(make_unique<frc::DigitalInput>(2));
    sensor_reader.set_proximal_potentiometer(make_unique<frc::AnalogInput>(2));

    sensor_reader.set_distal_encoder(make_encoder(2));
    sensor_reader.set_distal_absolute_pwm(make_unique<frc::DigitalInput>(3));
    sensor_reader.set_distal_potentiometer(make_unique<frc::AnalogInput>(3));

    sensor_reader.set_right_intake_encoder(make_encoder(5));
    sensor_reader.set_right_intake_absolute_pwm(
        make_unique<frc::DigitalInput>(7));
    sensor_reader.set_right_intake_potentiometer(
        make_unique<frc::AnalogInput>(1));
    sensor_reader.set_right_intake_spring_angle(
        make_unique<frc::AnalogInput>(5));
    sensor_reader.set_right_intake_cube_detector(
        make_unique<frc::DigitalInput>(1));

    sensor_reader.set_left_intake_encoder(make_encoder(3));
    sensor_reader.set_left_intake_absolute_pwm(
        make_unique<frc::DigitalInput>(4));
    sensor_reader.set_left_intake_potentiometer(
        make_unique<frc::AnalogInput>(0));
    sensor_reader.set_left_intake_spring_angle(
        make_unique<frc::AnalogInput>(4));
    sensor_reader.set_left_intake_cube_detector(
        make_unique<frc::DigitalInput>(0));

    sensor_reader.set_claw_beambreak(make_unique<frc::DigitalInput>(8));
    sensor_reader.set_box_back_beambreak(make_unique<frc::DigitalInput>(9));

    sensor_reader.set_pwm_trigger(true);

    sensor_reader.set_lidar_lite_input(make_unique<frc::DigitalInput>(22));
    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop imu_event_loop(&config.message());
    auto imu_trigger = make_unique<frc::DigitalInput>(5);
    ::frc971::wpilib::ADIS16448 imu(&imu_event_loop, frc::SPI::Port::kOnboardCS1,
                                    imu_trigger.get());
    imu.SetDummySPI(frc::SPI::Port::kOnboardCS2);
    auto imu_reset = make_unique<frc::DigitalOutput>(6);
    imu.set_reset(imu_reset.get());
    AddLoop(&imu_event_loop);

    // While as of 2/9/18 the drivetrain Victors are SPX, it appears as though
    // they are identical, as far as DrivetrainWriter is concerned, to the SP
    // variety so all the Victors are written as SPs.

    // Thread 5.
    ::aos::ShmEventLoop output_event_loop(&config.message());

    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(2)), false);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(3)), true);

    SuperstructureWriter superstructure_writer(&output_event_loop);
    superstructure_writer.set_left_intake_elastic_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(4)));
    superstructure_writer.set_left_intake_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(5)));
    superstructure_writer.set_right_intake_elastic_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(7)));
    superstructure_writer.set_right_intake_rollers_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(6)));
    superstructure_writer.set_proximal_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(0)));
    superstructure_writer.set_distal_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(1)));
    superstructure_writer.set_hanger_victor(
        ::std::unique_ptr<::frc::VictorSP>(new ::frc::VictorSP(8)));

    AddLoop(&output_event_loop);

    // Thread 6.
    // This is a separate event loop because we want to run it at much lower
    // priority.
    ::aos::ShmEventLoop solenoid_writer_event_loop(&config.message());
    ::frc971::wpilib::BufferedPcm pcm;
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop, &pcm);
    solenoid_writer.set_left_drivetrain_shifter(pcm.MakeSolenoid(0));
    solenoid_writer.set_right_drivetrain_shifter(pcm.MakeSolenoid(1));
    solenoid_writer.set_claw(pcm.MakeSolenoid(2));
    solenoid_writer.set_arm_brakes(pcm.MakeSolenoid(3));
    solenoid_writer.set_hook(pcm.MakeSolenoid(4));
    solenoid_writer.set_forks(pcm.MakeSolenoid(5));
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace
}  // namespace wpilib
}  // namespace y2018

AOS_ROBOT_CLASS(::y2018::wpilib::WPILibRobot);
