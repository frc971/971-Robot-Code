#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Talon.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#undef ERROR

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2014_bot3/control_loops/rollers/rollers.h"
#include "y2014_bot3/control_loops/rollers/rollers_output_generated.h"
#include "y2014_bot3/control_loops/rollers/rollers_position_generated.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::frc971::wpilib::BufferedPcm;
using ::frc971::wpilib::BufferedSolenoid;
using ::frc971::wpilib::LoopOutputHandler;
using ::frc971::wpilib::JoystickSender;
using ::frc971::wpilib::GyroSender;
using aos::make_unique;

namespace y2014_bot3 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2014_bot3::control_loops::drivetrain::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         ::y2014_bot3::control_loops::drivetrain::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

// Reads in our inputs. (sensors, voltages, etc.)
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        rollers_position_sender_(
            event_loop
                ->MakeSender<::y2014_bot3::control_loops::rollers::Position>(
                    "/rollers")),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                    "/drivetrain")) {}

  void RunIteration() {
    // Drivetrain
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();

      frc971::control_loops::drivetrain::Position::Builder position_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();
      position_builder.add_right_encoder(
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw()));
      position_builder.add_left_encoder(
          drivetrain_translate(drivetrain_left_encoder_->GetRaw()));
      position_builder.add_left_speed(
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));
      position_builder.add_right_speed(drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod()));

      builder.Send(position_builder.Finish());
    }

    // Rollers
    {
      auto builder = rollers_position_sender_.MakeBuilder();
      builder.Send(
          builder.MakeBuilder<control_loops::rollers::Position>().Finish());
    }
  }

 private:
  ::aos::Sender<::y2014_bot3::control_loops::rollers::Position>
      rollers_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
};

// Writes out our pneumatic outputs.
class SolenoidWriter {
 public:
  SolenoidWriter(::aos::ShmEventLoop *event_loop,
                 const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::drivetrain::Output>(
                    "/drivetrain")),
        rollers_(
            event_loop
                ->MakeFetcher<::y2014_bot3::control_loops::rollers::Output>(
                    "/rollers")),
        pneumatics_to_log_sender_(
            event_loop->MakeSender<::frc971::wpilib::PneumaticsToLog>("/aos")) {
    event_loop->set_name("Solenoids");
    event_loop->SetRuntimeRealtimePriority(27);

    event_loop->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                              ::std::chrono::milliseconds(20),
                              ::std::chrono::milliseconds(1));
  }

  void set_pressure_switch(
      ::std::unique_ptr<::frc::DigitalInput> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<::frc::Relay> compressor_relay) {
    compressor_relay_ = ::std::move(compressor_relay);
  }

  void set_drivetrain_left(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_left_ = ::std::move(s);
  }

  void set_drivetrain_right(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_right_ = ::std::move(s);
  }

  void set_rollers_front(::std::unique_ptr<BufferedSolenoid> s) {
    rollers_front_ = ::std::move(s);
  }

  void set_rollers_back(::std::unique_ptr<BufferedSolenoid> s) {
    rollers_back_ = ::std::move(s);
  }

  void Loop(const int iterations) {
    if (iterations != 1) {
      AOS_LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    // Drivetrain
    {
      drivetrain_.Fetch();
      if (drivetrain_.get()) {
        drivetrain_left_->Set(drivetrain_->left_high());
        drivetrain_right_->Set(drivetrain_->right_high());
      }
    }

    // Intake
    {
      rollers_.Fetch();
      if (rollers_.get()) {
        rollers_front_->Set(rollers_->front_extended());
        rollers_back_->Set(rollers_->back_extended());
      }
    }

    // Compressor
    {
      auto builder = pneumatics_to_log_sender_.MakeBuilder();

      ::frc971::wpilib::PneumaticsToLog::Builder to_log_builder =
          builder.MakeBuilder<frc971::wpilib::PneumaticsToLog>();

      {
        // Refill if pneumatic pressure goes too low.
        const bool compressor_on = !pressure_switch_->Get();
        to_log_builder.add_compressor_on(compressor_on);
        if (compressor_on) {
          compressor_relay_->Set(::frc::Relay::kForward);
        } else {
          compressor_relay_->Set(::frc::Relay::kOff);
        }
      }

      pcm_->Flush();
      to_log_builder.add_read_solenoids(pcm_->GetAll());
      builder.Send(to_log_builder.Finish());
    }
  }

 private:
  const ::std::unique_ptr<BufferedPcm> &pcm_;

  ::std::unique_ptr<BufferedSolenoid> drivetrain_left_, drivetrain_right_;
  ::std::unique_ptr<BufferedSolenoid> rollers_front_, rollers_back_;

  ::std::unique_ptr<::frc::DigitalInput> pressure_switch_;
  ::std::unique_ptr<::frc::Relay> compressor_relay_;

  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output> drivetrain_;
  ::aos::Fetcher<::y2014_bot3::control_loops::rollers::Output> rollers_;
  aos::Sender<::frc971::wpilib::PneumaticsToLog> pneumatics_to_log_sender_;
};

// Writes out rollers voltages.
class RollersWriter : public LoopOutputHandler<
                          ::y2014_bot3::control_loops::rollers::Output> {
 public:
  RollersWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<
            ::y2014_bot3::control_loops::rollers::Output>(event_loop,
                                                          "/rollers") {}

  void set_rollers_front_intake_talon(::std::unique_ptr<::frc::Talon> t_left,
                                      ::std::unique_ptr<::frc::Talon> t_right) {
    rollers_front_left_intake_talon_ = ::std::move(t_left);
    rollers_front_right_intake_talon_ = ::std::move(t_right);
  }

  void set_rollers_back_intake_talon(::std::unique_ptr<::frc::Talon> t_left,
                                     ::std::unique_ptr<::frc::Talon> t_right) {
    rollers_back_left_intake_talon_ = ::std::move(t_left);
    rollers_back_right_intake_talon_ = ::std::move(t_right);
  }

  void set_rollers_low_goal_talon(::std::unique_ptr<::frc::Talon> t) {
    rollers_low_goal_talon_ = ::std::move(t);
  }

 private:
  virtual void Write(const ::y2014_bot3::control_loops::rollers::Output
                         &output) override {
    rollers_front_left_intake_talon_->SetSpeed(output.front_intake_voltage() /
                                               12.0);
    rollers_front_right_intake_talon_->SetSpeed(
        -(output.front_intake_voltage() / 12.0));
    rollers_back_left_intake_talon_->SetSpeed(output.back_intake_voltage() /
                                              12.0);
    rollers_back_right_intake_talon_->SetSpeed(
        -(output.back_intake_voltage() / 12.0));
    rollers_low_goal_talon_->SetSpeed(output.low_goal_voltage() / 12.0);
  }

  virtual void Stop() override {
    AOS_LOG(WARNING, "Intake output too old\n");
    rollers_front_left_intake_talon_->SetDisabled();
    rollers_front_right_intake_talon_->SetDisabled();
    rollers_back_left_intake_talon_->SetDisabled();
    rollers_back_right_intake_talon_->SetDisabled();
    rollers_low_goal_talon_->SetDisabled();
  }

  ::std::unique_ptr<::frc::Talon> rollers_front_left_intake_talon_,
      rollers_back_left_intake_talon_, rollers_front_right_intake_talon_,
      rollers_back_right_intake_talon_, rollers_low_goal_talon_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<::frc::Encoder> make_encoder(int index) {
    return make_unique<::frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                       ::frc::Encoder::k4X);
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
    // Sensors
    ::aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    SensorReader sensor_reader(&sensor_reader_event_loop);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(4));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(5));
    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop gyro_event_loop(&config.message());
    GyroSender gyro_sender(&gyro_event_loop);
    AddLoop(&gyro_event_loop);

    // Thread 5.
    // Outputs
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(5)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(2)), false);

    RollersWriter rollers_writer(&output_event_loop);
    rollers_writer.set_rollers_front_intake_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(3)),
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(7)));
    rollers_writer.set_rollers_back_intake_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(1)),
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(6)));

    rollers_writer.set_rollers_low_goal_talon(
        ::std::unique_ptr<::frc::Talon>(new ::frc::Talon(4)));
    AddLoop(&output_event_loop);

    // Thread 6.
    ::aos::ShmEventLoop solenoid_writer_event_loop(&config.message());
    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop, pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(5));
    solenoid_writer.set_rollers_front(pcm->MakeSolenoid(2));
    solenoid_writer.set_rollers_back(pcm->MakeSolenoid(4));

    // Don't change the following IDs.
    solenoid_writer.set_pressure_switch(make_unique<::frc::DigitalInput>(9));
    solenoid_writer.set_compressor_relay(make_unique<::frc::Relay>(0));
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2014_bot3

AOS_ROBOT_CLASS(::y2014_bot3::wpilib::WPILibRobot);
